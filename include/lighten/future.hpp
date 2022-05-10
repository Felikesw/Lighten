#ifndef __LIGHTEN__ASYNC__STATE__HPP
#define __LIGHTEN__ASYNC__STATE__HPP

#include <exception>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <lighten/ref.hpp>
#include <lighten/task.hpp>

namespace lighten {

template<typename R, typename ...ArgTypes > class packaged_task;
template <typename T> class future;
template <typename T> class promise;

template<typename C>
struct is_future{
  private:
  template<typename T>
  static constexpr auto check(T*) -> typename std::is_same<future<typename T::type>, T>::type;

  template<typename>
  static constexpr std::false_type check(...);

  typedef decltype(check<C>(0)) type;

  public:
  static constexpr bool value = type::value;
};

template<typename T>
inline constexpr bool is_future_v = is_future<T>::value;

template<typename C>
struct is_referenceable_ref_ptr{
  private:
  template<typename T>
  static constexpr auto check(T*) -> typename std::conjunction<is_ref_ptr<T>, is_referenceable<typename T::type>>::type;

  template<typename>
  static constexpr std::false_type check(...);

  typedef decltype(check<C>(0)) type;

  public:
  static constexpr bool value = type::value;
};


template<typename C, bool has_type = (is_future_v<C> || is_referenceable_ref_ptr<C>::value)>
struct extract_future_type{
  typedef typename C::type type;
};

template<typename C>
struct extract_future_type<C, false> {
  typedef C type;
};


namespace future_internals {

template <typename T> class state;

class state_base {
  template <typename T>
  friend class state;
  protected:
  enum {
    OPEN,
    DONE,
    EXCEPT,
    FORWARD,
    CALLBACK,
    STATE_MAX
  };
  struct wait_state {
    std::mutex _mutex;
    std::condition_variable cv;
  }; 

  class callback_base {
    public:
    virtual ~callback_base() {}
    virtual void invoke(state_base *) = 0;
  };

  template<typename T, class F>
  class callback_wrapper: public callback_base {
    F func;
    public:
    callback_wrapper(F &&f): func(std::forward<F>(f)){};
    void invoke(state_base *sb) override {
      func(static_cast<state<T>*>(sb));
    }
  };

  static const uintptr_t STATE_MASK = 3;
  static inline int STATE(uintptr_t p) {
    return (p<STATE_MAX)?(int)p:(p & STATE_MASK)+1;
  }
  inline int STATE() {
    return STATE(data);
  }
  static inline uintptr_t PTR(uintptr_t p) {
    return (p<STATE_MAX)?0:((p)&(~STATE_MASK));
  }
  inline uintptr_t PTR() {
    return PTR(data);
  }

  std::atomic_uintptr_t data;

  unsigned char get_state() {
    uintptr_t ptr = data;
    return STATE(ptr);
  }

  state_base *forward_state() {
    uintptr_t ptr = data;
    if(STATE(ptr) == FORWARD && PTR(ptr)){
      state_base *state_p = reinterpret_cast<state_base *>(PTR(ptr));
      data = FORWARD;

      return state_p;
    }
    else {
      throw std::exception();
    }
  }

  int set_callback(callback_base *cb){
    uintptr_t ptr = data;

    do {
      switch(STATE(ptr)){
        case OPEN: {
          if(data.compare_exchange_strong(ptr, (reinterpret_cast<uintptr_t>(cb) | (CALLBACK - 1)))){
            return 0;
          }
        }break;
        case CALLBACK: {
          delete cb;
          throw std::exception();
          return -1;
        }break;
        case FORWARD: {
          return forward_state()->set_callback(cb);
        }break;
        default: {
          cb->invoke(this);
          delete cb;
          return 1;
        }
      }
    }while(true);
  }

  void set_value_ptr(uintptr_t value_ptr){
    uintptr_t ptr = data;

    do {
      switch(STATE(ptr)){
        case OPEN: {
          if(data.compare_exchange_strong(ptr, value_ptr)){
            return;
          }
        }break;
        case CALLBACK: {
          callback_base *cb = reinterpret_cast<callback_base *>(PTR(ptr));
          if(data.compare_exchange_strong(ptr, value_ptr)){
            cb->invoke(this);
            delete cb;
            return;
          }

        }break;
        default: {
          throw std::exception();
        }
      }
    }while(true);
  }

  void rethrow_except() {
    uintptr_t ptr = data;

    if(STATE(ptr) == EXCEPT && PTR(ptr)) {
      std::exception_ptr *eptr_p = reinterpret_cast<std::exception_ptr *>(PTR(ptr));
      data = EXCEPT;

      std::exception_ptr eptr = std::move(*eptr_p);
      delete eptr_p;

      rethrow_exception(eptr);
    }
  }

  template<typename T>
  T *get_value_ptr() {
    uintptr_t ptr = data;

    if(PTR(ptr)) {
      switch(STATE(ptr)){
        case DONE:
          data = DONE;
          return reinterpret_cast<T*>(PTR(ptr));
        case EXCEPT:
          rethrow_except();
          break;
      }
    }

    throw std::exception();
  }

  void set_forward(state_base *to){
    uintptr_t ptr = data;

    do {
      switch(STATE(ptr)){
        case OPEN: {
          if(data.compare_exchange_strong(ptr, (reinterpret_cast<uintptr_t>(to) | (FORWARD - 1)))){
            return;
          }
        }break;
        case CALLBACK: {
          callback_base *cb = reinterpret_cast<callback_base *>(PTR(ptr));
          if(data.compare_exchange_strong(ptr, FORWARD)){
            to->set_callback(cb);
            return;
          }
        }break;
        default: {
          throw std::exception();
        }
      }
    }while(true);
  }


  class wait_callback: public callback_base {
    protected:
    wait_state *ws;
    public:
    wait_callback(wait_state *ws): ws(ws) {}
    void invoke(state_base *sb) override {
      std::lock_guard lock(ws->_mutex);
      ws->cv.notify_all();
    }
  };


  public:
  void wait() {
    wait_state *ws = new wait_state();
    if(set_callback(new wait_callback(ws)) == 0) {
      std::unique_lock lock(ws->_mutex);
      ws->cv.wait(lock, [this]{ return get_state() != CALLBACK;});
    }
    delete ws;
  }

  void set_exception(std::exception_ptr eptr) {
    uintptr_t ex_ptr = reinterpret_cast<uintptr_t>(new std::exception_ptr(eptr)) | (EXCEPT - 1);
    uintptr_t ptr = data;

    do {
      switch(STATE(ptr)){
        case OPEN: {
          if(data.compare_exchange_strong(ptr, ex_ptr)){
            return;
          }
        }break;
        case CALLBACK: {
          callback_base *cb = reinterpret_cast<callback_base *>(PTR(ptr));
          if(data.compare_exchange_strong(ptr, ex_ptr)){
            cb->invoke(this);
            delete cb;
            return;
          }
        }
        default: {
          throw std::exception();
        }
      }
    }while(true);
  }

  state_base(): data(0) {};

  ~state_base() {
    if(PTR()){
      switch(STATE()){
        case FORWARD:
          forward_state();
          break;
        case EXCEPT:
          rethrow_except();
          break;
      }
    }
  }
};


template<typename T>
class state: public state_base {
  template<typename U>
  friend class future;
  friend class promise<T>;
  template<typename R, typename ...ArgTypes >
  friend class lighten::packaged_task;
  protected:

  state() {};
  public:
  state(const state &) = delete;
  state(state &&s)/*: state_base(std::move(s)) */{
    data = s.data.exchange(0);
  }
  ~state() {
    if(STATE() == DONE && PTR()) {
      T *value_ptr = get_value_ptr<T>();
      if(value_ptr)delete value_ptr;
    }
  }

  void forward(state<T> *to){
    state_base::set_forward(to);
  }

  void set_value(const T &res){
    T *value_ptr = new T(res);
    state_base::set_value_ptr(reinterpret_cast<uintptr_t>(value_ptr));
  }

  void set_value(T &&res){
    T *value_ptr = new T(std::move(res));
    state_base::set_value_ptr(reinterpret_cast<uintptr_t>(value_ptr));
  }

  void wait() {
    state_base::wait();
  }

  T get() {
    wait();
    if(get_state() == FORWARD) {
      return static_cast<state *>(forward_state())->get();
    }

    T *value_p = get_value_ptr<T>();
    T value = std::move(*value_p);

    delete value_p;
    return value;
  }

  template<typename F>
  void on_complete(F&& func) {
    set_callback(new callback_wrapper<T, F>(std::forward<F>(func)));
  }
};

template<typename T>
class state<T&>: public state_base {
  template<typename U>
  friend class future;
  friend class promise<T&>;
  template<typename R, typename ...ArgTypes >
  friend class lighten::packaged_task;

  protected:
  state() {};

  public:
  state(const state &) = delete;
  state(state &&s)/*:state_base(std::move(s))*/{
    data = s.data.exchange(0);
  }

  void forward(ref_ptr<state> to){
    state_base::set_forward(to);
  }
 
  void set_value(T &res){
    T *value_ptr = &res;
    state_base::set_value_ptr(reinterpret_cast<uintptr_t>(value_ptr));
  }

  void wait() {
    state_base::wait();
  }

  T& get() {
    wait();
    if(get_state() == FORWARD) {
      return static_cast<state *>(forward_state())->get();
    }
    return *get_value_ptr<T>();
  }

  template<typename F>
  void on_complete(F&& func) {
    set_callback(ref_new<callback_wrapper<T&, F>>(std::forward<F>>(func)));
  }
};

template<typename T>
class state<T*>: public state_base {
  template<typename U>
  friend class future;
  friend class promise<T*>;
  //template <typename RT, class F, typename ...ArgTypes>
  //friend class task_wrapper;
  template<typename R, typename ...ArgTypes >
  friend class lighten::packaged_task;

  protected:
  state() {};

  public:
  state(const state &) = delete;
  state(state &&s)/*:state_base(std::move(s))*/{
    data = s.data.exchange(0);
  }

  void forward(ref_ptr<state> to){
    state_base::set_forward(to);
  }
 
  void set_value(T *ptr){
    state_base::set_value_ptr(reinterpret_cast<uintptr_t>(ptr));
  }

  void wait() {
    state_base::wait();
  }

  T *get() {
    wait();
    if(get_state() == FORWARD) {
      return static_cast<state *>(forward_state())->get();
    }
    return get_value_ptr<T*>();
  }

  template<typename F>
  void on_complete(F&& func) {
    set_callback(ref_new<callback_wrapper<T*, F>>(std::forward<F>(func)));
  }
};

template<typename T>
class state<std::enable_if<!is_referenceable_v<T>, ref_ptr<T>>>: public state_base {
  template<typename U>
  friend class future;
  friend class promise<ref_ptr<T>>;
  template<typename R, typename ...ArgTypes >
  friend class lighten::packaged_task;

  protected:
  state() {};
  ~state() {
    if(STATE() == DONE && PTR()){
      lighten::ref<T> *p = get_value_ptr<lighten::ref<T>>();
      if(p)delete p;
    }
  }

  public:
  state(const state &) = delete;
  state(state &&s)/*:state_base(std::move(s))*/{
    data = s.data.exchange(0);
  }

  void forward(ref_ptr<state> to){
    state_base::set_forward(to);
  }
 
  void set_value(ref_ptr<T> ptr){
    //lighten::ref<T> *ref = ptr.get_ref();
    //state_base::set_value_ptr(reinterpret_cast<uintptr_t>(ref));
    state_base::set_value_ptr(ptr.move_ptr());
  }

  void wait() {
    state_base::wait();
  }

  ref_ptr<T> get() {
    wait();
    if(get_state() == FORWARD) {
      return static_cast<state *>(forward_state())->get();
    }
    lighten::ref<T> *ref = get_value_ptr<lighten::ref<T>>();
    ref_ptr<T> res(ref);

    return res;
  }

  template<typename F>
  void on_complete(F&& func) {
    set_callback(new callback_wrapper<ref_ptr<T>, F>(std::forward<F>(func)));
  }
};

template<typename T>
class state<enable_if_referenceable<T>>: public state_base {
  template<typename U>
  friend class future;
  friend class promise<T>;
  template<typename R, typename ...ArgTypes >
  friend class lighten::packaged_task;

  protected:
  state() {};
  ~state() {
    if(STATE() == DONE && PTR()){
      lighten::ref<T> *p = get_value_ptr<lighten::ref<T>>();
      if(p)delete p;
    }
  }

  public:
  state(const state &) = delete;
  state(state &&s)/*:state_base(std::move(s))*/{
    data = s.data.exchange(0);
  }

  void forward(ref_ptr<state> to){
    state_base::set_forward(to);
  }
 
  void set_value(ref_ptr<T> ptr){
    //lighten::ref<T> *ref = ptr.get_ref();
    //state_base::set_value_ptr(reinterpret_cast<uintptr_t>(ref));
    state_base::set_value_ptr(ptr.move_ptr());
  }

  void wait() {
    state_base::wait();
  }

  ref_ptr<T> get() {
    wait();
    if(get_state() == FORWARD) {
      return static_cast<state *>(forward_state())->get();
    }
    lighten::ref<T> *ref = get_value_ptr<lighten::ref<T>>();
    ref_ptr<T> res(ref);

    return res;
  }

  template<typename F>
  void on_complete(F&& func) {
    set_callback(new callback_wrapper<T, F>(std::forward<F>(func)));
  }
};


template<>
class state<void>: public state_base {
  template<typename U>
  friend class future;
  friend class promise<void>;
  template<typename R, typename ...ArgTypes >
  friend class lighten::packaged_task;

  protected:
  state() {};

  public:
  state(const state &) = delete;
  state(state &&s)/*: state_base(std::move(s))*/{
    data = s.data.exchange(0);
  }

  void forward(state *to){
    state_base::set_forward(to);
  }
 
  void set_value(){
    state_base::set_value_ptr(reinterpret_cast<uintptr_t>(this));
  }

  void wait() {
    state_base::wait();
  }

  void get() {
    wait();
    if(get_state() == FORWARD) {
      static_cast<state *>(forward_state())->get();
    }
    else {
      get_value_ptr<void>();
    }
  }

  template<typename F>
  void on_complete(F&& func) {
    set_callback(new callback_wrapper<void, F>(std::forward<F>(func)));
  }
};

}

template<typename R, typename ...V>
class packaged_task {
  protected:
  template <typename RT, typename ...ArgTypes>
  class task_base{
    public:
      virtual void invoke(ArgTypes&&... args) = 0;
      virtual ~task_base() = default;
  };

  template <typename RT, class F, typename ...ArgTypes>
  class task_wrapper: public task_base<RT, ArgTypes...> {
    future_internals::state<RT> *state;
    F func;

    template<class C, class P, class T, typename ...A>
    void invoke_memb(P C::* f, T &&t, A&&... args) {
      try {
        if constexpr (is_future_v<std::invoke_result_t<decltype(f), C, A...>>){
          if constexpr (std::is_base_of_v<C, std::decay_t<T>>){
            state->forward((std::forward<T>(t).*f)(std::forward<A>(args)...).state);
          }
          else {
            state->forward((*std::forward<T>(t).*f)(std::forward<A>(args)...).state);
          }
        }
        else if constexpr (std::is_void_v<RT>){
          if constexpr (std::is_base_of_v<C, std::decay_t<T>>){
            (std::forward<T>(t).*f)(std::forward<A>(args)...);
          }
          else {
            (*std::forward<T>(t).*f)(std::forward<A>(args)...);
          }
          state->set_value();
        }
        else {
          if constexpr (std::is_base_of_v<C, std::decay_t<T>>){
            state->set_value((std::forward<T>(t).*f)(std::forward<A>(args)...));
          }
          else {
            state->set_value((*std::forward<T>(t).*f)(std::forward<A>(args)...));
          }
        }
      }catch(...){
        state->set_exception(std::current_exception());
      }
    }

    public:
    task_wrapper(future_internals::state<RT> *state, F &&f): state(state), func(std::forward<F>(f)){
    }

    void invoke(ArgTypes&&... args ) override{
      if constexpr(std::is_member_function_pointer_v<F>){
        invoke_memb(func, state, std::forward<ArgTypes>(args)...);
      }
      else {
        try {
          if constexpr(is_future_v<std::invoke_result_t<F, ArgTypes...>>){
            future<RT> fut = func(std::forward<ArgTypes>(args)...);
            state->forward(fut.state);
          }
          else if constexpr(std::is_void_v<RT>) {
            func(std::forward<ArgTypes>(args)...);
            state->set_value();
          }
          else {
            state->set_value(func(std::forward<ArgTypes>(args)...));
          }
        }catch(...) {
          state->set_exception(std::current_exception());
        }
      }
    }
  };

  public:
  void operator()(V... args) {
    task->invoke(std::forward<V>(args)...);
    delete task;
    task = nullptr;
  }

  template<class F>
  packaged_task(F&& f): 
    state(new future_internals::state<R>()),
    task(new task_wrapper<R, F, V...>(state, std::forward<F>(f))) {};

  future<R> get_future() {
    return future<R>(std::move(state));
  }

  packaged_task( const packaged_task& ) = delete;
  packaged_task( packaged_task&& rhs ) noexcept = default;
  ~packaged_task() {
    if(state){
      delete state;
    }

    if(task){
      delete task;
    }
  }

  packaged_task& operator=( const packaged_task& ) = delete;
  packaged_task& operator=( packaged_task&& rhs ) noexcept = default;

  bool valid() const noexcept {
    return this->state;
  }

  protected:
  future_internals::state<R> *state;
  task_base<R, V...> *task;
};

template<typename R, typename ...V>
class packaged_task<R(V...)>: public packaged_task<R, V...> {
  public:
  template<class F>
  packaged_task(F&& f): packaged_task<R,V...>(std::forward<F>(f)){}
  packaged_task( const packaged_task& ) = delete;
  packaged_task( packaged_task&& rhs ) noexcept = default;
};

class executor {
  protected:
  ref_ptr<task::handler> handler;
  static ref_ptr<task::handler> default_handler;
  public:
  template<typename F, typename ...V>
  void run(F &&f, V&&...args) const {
    handler->add_task(task(std::forward<F>(f), std::forward<V>(args)...));
  }

  template<typename F, typename ...V>
  future<typename extract_future_type<std::result_of_t<F(V...)>>::type> execute(F &&f, V&&...args) {
    typedef typename extract_future_type<std::result_of_t<F(V...)>>::type R;
    packaged_task<R, V...> tpack(std::forward<F>(f));
    future<R> fut = tpack.get_future();
    handler->add_task(task([tpack = std::move(tpack)](V...args)mutable{
        tpack(std::forward<V>(args)...);
      }, 
      std::forward<V>(args)...)
    );

    return fut;
  }

  executor(): handler(executor::default_handler){}
  executor(ref_ptr<task::handler> handler):handler(handler){}
};


template<typename T>
class future {
  friend class promise<T>;

  template<typename U>
  friend class future;

  template <typename RT, class F, typename ...ArgTypes>
  friend  class task_wrapper;

  template<typename R, typename ...V>
  friend class lighten::packaged_task; 
  protected:
  future_internals::state<T> *state;
  future(future_internals::state<T> *state):state(state){
    if(!state)abort();
  }

  public:
  typedef T type;

  future(){}
  future(const future &) = delete;
  future(future &&) = default; 

  ~future(){
    if(state)delete state;
  }

  T get() { 
    T value = state->get();
    delete state;
    state = nullptr;
    return value;
  }

  operator T () { return get(); }

  template<typename F>
  future<typename extract_future_type<std::result_of_t<F(future)>>::type> then(F &&func)
  {
    typedef typename extract_future_type<std::result_of_t<F(future)>>::type R;
    packaged_task<R, future> callback(std::forward<F>(func));
    future<R> fut = callback.get_future();

    this->state->on_complete([callback = std::move(callback)](future_internals::state<T> *state) mutable {
      callback(future(state));
    });

    this->state = nullptr;

    return fut;
  }

  template<typename F>
  future<typename extract_future_type<std::result_of_t<F(future)>>::type> then(const executor &ex, F &&func)
  {
    typedef typename extract_future_type<std::result_of_t<F(future)>>::type R;
    packaged_task<R, future> callback(std::forward<F>(func));
    future<R> fut = callback.get_future();

    this->state->on_complete([ex = ex, callback = std::move(callback)](future_internals::state<T> *state) mutable {
      ex.run(std::move(callback), future(state));
    });

    this->state = nullptr;

    return fut;
  }
};

template<typename T>
class future<T&> {
  friend class promise<T&>;

  template<typename U>
  friend class future;
  protected:
  future_internals::state<T&> state;
  future(future_internals::state<T&> &state):state(state){
    if(!state)abort();
  }

  public:
  typedef T& type;

  future(){}
  future(const future &) = delete;
  future(future &&) = default; 

  ~future(){
    if(state)delete state;
  }

  T& get() { 
    T& value = state->get();
    //state.reset();
    delete state;
    state = nullptr;
    return value;
  }

  operator T& () { return get(); }

  template<typename F>
  future<typename extract_future_type<std::result_of_t<F(future)>>::type> then(F &&func)
  {
    typedef typename extract_future_type<std::result_of_t<F(future)>>::type R;
    packaged_task<R, future> callback(std::forward<F>(func));
    future<R> fut = callback.get_future();

    this->state->on_complete([callback = std::move(callback)](future_internals::state<T> *state) mutable {
      callback(future(state));
    });

    this->state = nullptr;

    return fut;
  }

  template<typename F>
  future<typename extract_future_type<std::result_of_t<F(future)>>::type> then(const executor &ex, F &&func)
  {
    typedef typename extract_future_type<std::result_of_t<F(future)>>::type R;
    packaged_task<R, future> callback(std::forward<F>(func));
    future<R> fut = callback.get_future();

    this->state->on_complete([ex = ex, callback = std::move(callback)](future_internals::state<T> *state) mutable {
      ex.run(std::move(callback), future(state));
    });

    this->state = nullptr;

    return fut;
  }
};


template<typename T>
class future<enable_if_referenceable<T>>
{
  friend class promise<T>;

  template<typename U>
  friend class future;

  template <typename RT, class F, typename ...ArgTypes>
  friend  class task_wrapper;

  template<typename R, typename ...V>
  friend class lighten::packaged_task; 
  protected:
  future_internals::state<T> *state;
  future(future_internals::state<T> *&state):state(state){
    if(!state)abort();
  }

  public:
  typedef T type;

  future(){}
  future(const future &) = delete;
  future(future &&) = default; 

  ~future(){
    if(state)delete state;
  }

  ref_ptr<T> get() { 
    ref_ptr<T> ptr = state->get();
    //state.reset();
    delete state;
    state = nullptr;
    return ptr;
  }

  operator ref_ptr<T> () { return get(); }

  template<typename F>
  future<typename extract_future_type<std::result_of_t<F(future)>>::type> then(F &&func)
  {
    typedef typename extract_future_type<std::result_of_t<F(future)>>::type R;
    packaged_task<R, future> callback(std::forward<F>(func));
    future<R> fut = callback.get_future();

    this->state->on_complete([callback = std::move(callback)](future_internals::state<T> *state) mutable {
      callback(future(state));
    });

    this->state = nullptr;

    return fut;
  }

  template<typename F>
  future<typename extract_future_type<std::result_of_t<F(future)>>::type> then(const executor &ex, F &&func)
  {
    typedef typename extract_future_type<std::result_of_t<F(future)>>::type R;
    packaged_task<R, future> callback(std::forward<F>(func));
    future<R> fut = callback.get_future();

    this->state->on_complete([ex = ex, callback = std::move(callback)](future_internals::state<T> *state) mutable {
      ex.run(std::move(callback), future(state));
    });

    this->state = nullptr;

    return fut;
  }
};


template<>
class future<void> {
  friend class promise<void>;
  template<typename U>
  friend class future;
  template<typename R, typename ...V>
  friend class lighten::packaged_task; 
  protected:
  future_internals::state<void> *state;
  future(future_internals::state<void> *state):state(state){
    if(!state)abort();
  }

  public:
  typedef void type;

  future(){}
  future(const future &) = delete;
  future(future &&) = default; 

  ~future(){
    if(state)delete state;
  }

  void get() {
    state->get();
    //state.reset();
    delete state;
    state = nullptr;
  }

  template<typename F>
  future<typename extract_future_type<std::result_of_t<F(future)>>::type> then( F && func)
  {
    typedef typename extract_future_type<std::result_of_t<F(future)>>::type R;
    packaged_task<R, future> callback(std::forward<F>(func));
    future<R> fut = callback.get_future();

    this->state->on_complete([callback = std::move(callback)](future_internals::state<void> *state) mutable {
      callback(future(state));
    });

    this->state = nullptr;

    return fut;
  }

  template<typename F>
  future<typename extract_future_type<std::result_of_t<F(future)>>::type> then(const executor &ex, F &&func)
  {
    typedef typename extract_future_type<std::result_of_t<F(future)>>::type R;
    packaged_task<R, future> callback(std::forward<F>(func));
    future<R> fut = callback.get_future();

    this->state->on_complete([ex = ex, callback = std::move(callback)](future_internals::state<void> *state) mutable {
      ex.run(std::move(callback), future(state));
    });

    this->state = nullptr;

    return fut;
  }

  template<typename F>
  future<typename extract_future_type<std::result_of_t<F()>>::type> then( F && func)
  {
    typedef typename extract_future_type<std::result_of_t<F()>>::type R;
    packaged_task<R, future> callback([func = std::forward<F>(func)](future fut){
      fut.get();
      func();
    });
    future<R> fut = callback.get_future();

    this->state->on_complete([callback = std::move(callback)](future_internals::state<void> *state) mutable {
      callback(future(state));
    });

    this->state = nullptr;

    return fut;
  }

  template<typename F>
  future<typename extract_future_type<std::result_of_t<F()>>::type> then(const executor &ex, F && func)
  {
    typedef typename extract_future_type<std::result_of_t<F()>>::type R;
    packaged_task<R, future> callback([func = std::forward<F>(func)](future fut){
      fut.get();
      func();
    });
    future<R> fut = callback.get_future();

    this->state->on_complete([ex = ex, callback = std::move(callback)](future_internals::state<void> *state) mutable {
      ex.run(callback, future(state));
    });

    this->state = nullptr;

    return fut;
  }
};

class promise_not_fullfill: public std::exception {
  const char *what() const throw ()
  {
    return "promise not fullfill";
  }
};

class promise_already_fullfilled: public std::exception {
  const char *what() const throw ()
  {
    return "promise already fullfilled";
  }
};


template<typename T>
class promise
{
  protected:
  future_internals::state<T> *state;
  void check_state() {
    if(state) {
      throw promise_not_fullfill();
    }
  }
  public:
  future<T> get_future() {
    return future<T>(state);
  }

  void set_value(const T &value) {
    if(!state){
      throw promise_already_fullfilled();
    }
    else {
      auto state = std::move(this->state);
      state->set_value(value);
      //state.reset();
    }
  }

  void set_value(T &&value) {
    if(!state){
      throw promise_already_fullfilled();
    }
    else {
      auto state = std::move(this->state);
      state->set_value(std::move(value));
      //state.reset();
    }
  }

  void set_exception (std::exception_ptr eptr = std::current_exception()) {
    if(!state){
      throw promise_already_fullfilled();
    }
    else {
      auto state = std::move(this->state);
      state->set_exception(eptr);
      //state.reset();
    }
  }

  template<class E>
  void set_exception(const E &e) {
    set_exception(std::make_exception_ptr(e));
  }

  promise():state(new future_internals::state<T>()) {
  }
  
  ~promise() {
    check_state();
  }

  promise(const promise &) = delete;
  promise(promise &&s) {
    //fprintf(stderr, "promise<T> move contructor called\n");
    state = std::move(s.state);
  }

  promise &operator = (promise &&s) {
    //fprintf(stderr, "promise<T> move assignment called\n");
    if(!s.state) abort();
    state = std::move(s.state);
    //if(!state)abort();
    return *this;
  }
};

template<typename T>
class promise<T&>
{
  protected:
  future_internals::state<T&> *state;
  void check_state() {
    if(state) {
      throw promise_not_fullfill();
    }
  }
  public:
  future<T> get_future(){
    return future<T>(state);
  }

  void set_value(T &value) {
    if(!state){
      throw promise_already_fullfilled();
    }
    else {
      auto state = std::move(this->state);
      state->set_value(value);
      //state.reset();
    }
  }

  void set_exception (std::exception_ptr eptr = std::current_exception()) {
    if(!state){
      throw promise_already_fullfilled();
    }
    else {
      auto state = std::move(this->state);
      state->set_exception(eptr);
      //state.reset();
    }
  }

  template<class E>
  void set_exception(const E &e) {
    set_exception(std::make_exception_ptr(e));
  }

  promise():state(new future_internals::state<T&>()) {
  }
  
  ~promise() {
    check_state();
  }

  promise(const promise &) = delete;
  promise(promise &&s) {
    state = std::move(s.state);
  }

  promise &operator = (promise &&s) {
    state = std::move(s.state);
    return *this;
  }
};

template<typename T>
class promise<enable_if_referenceable<T>>
{
  protected:
  future_internals::state<T> *state;
  void check_state() {
    if(state) {
      throw promise_not_fullfill();
    }
  }
  public:
  future<T> get_future() {
    return future<T>(state);
  }

  void set_value(ref_ptr<T> value) {
    if(!state){
      throw promise_already_fullfilled();
    }
    else {
      auto state = std::move(this->state);
      state->set_value(value);
      //state.reset();
    }
  }

  void set_exception (std::exception_ptr eptr = std::current_exception()) {
    if(!state){
      throw promise_already_fullfilled();
    }
    else {
      auto state = std::move(this->state);
      state->set_exception(eptr);
      //state.reset();
    }
  }

  template<class E>
  void set_exception(const E &e) {
    set_exception(std::make_exception_ptr(e));
  }

  promise():state(new future_internals::state<T>()) {
  }
  
  ~promise() {
    check_state();
  }

  promise(const promise &) = delete;
  promise(promise &&s) {
    state = std::move(s.state);
  }

  promise &operator = (promise &&s) {
    state = std::move(s.state);
    return *this;
  }
  //promise(promise &&) = default; 
  //promise &operator = (promise &&) = default;
};

template<>
class promise<void>
{
  protected:
  future_internals::state<void> *state;

  public:
  future<void> get_future() {
    return future<void>(state);
  }

  void set_value(){
    if(!state){
      throw promise_already_fullfilled();
    }
    else {
      auto state = std::move(this->state);
      state->set_value();
      //state.reset();
    }
  }

  void set_exception (std::exception_ptr eptr = std::current_exception()){
    if(!state){
      throw promise_already_fullfilled();
    }
    else {
      auto state = std::move(this->state);
      state->set_exception(eptr);
      //state.reset();
    }
  }

  template<class E>
  void set_exception(const E &e){
    set_exception(std::make_exception_ptr(e));
  }

  promise():state(new future_internals::state<void>()) {
  }

  promise(const promise &) = delete;
    promise(promise &&s) {
    state = std::move(s.state);
  }

  promise &operator = (promise &&s) {
    state = std::move(s.state);
    return *this;
  }
  //promise(promise &&) = default;
  //promise &operator = (promise &&) = default;
};



}

#endif
