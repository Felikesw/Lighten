#ifndef __LIGHTEN__UTIL__REF__HPP
#define __LIGHTEN__UTIL__REF__HPP

#include <unistd.h>
#include <type_traits>
#include <exception>
#include <atomic>

//#define USE_GCC_ATOMIC_BUILTINS 1

namespace lighten {

template<class T> class ref_ptr;

namespace internals {
class ref_ptr_base;
}

class referenceable {
  template<class T>
  friend class ref_ptr;
  friend class internals::ref_ptr_base;
  private:
  std::atomic<int> ref_count;
  protected:
  int get_ref() {
    return ref_count++;
  }
  int put_ref() {
    int res = ref_count--;
    if(res == 0){
      delete this;
    }
    return res;
  }
  referenceable():ref_count(0) {}
  virtual ~referenceable() {}; 
};

namespace internals {

class ref_ptr_base {
  private:
#ifdef USE_GCC_ATOMIC_BUILTINS
  uintptr_t *m_ptr_p;
#else
	std::atomic_uintptr_t *m_ptr_p;
#endif

  class ref_ptr_lock_error: public std::exception {
    const char *what() const throw ()
    {
      return "reference pointer lock error";
    }
  };

  class ref_ptr_unlock_error: public std::exception {
    const char *what() const throw ()
    {
      return "reference pointer unlock error";
    }
  };


  inline bool atomic_compare_exchange_ptr(uintptr_t &expected, uintptr_t desired) const {
#ifdef USE_GCC_ATOMIC_BUILTINS
    return __atomic_compare_exchange_n(m_ptr_p, &expected, desired, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
 #else
    return m_ptr_p->compare_exchange_weak(expected, desired, std::memory_order_release, std::memory_order_relaxed);
#endif
  }

  static inline void YIELD() { usleep(1); }
  static const uintptr_t REF_PTR_LOCK  = 1;
  static const uintptr_t REF_PTR_MASK = ~REF_PTR_LOCK;
  static inline uintptr_t REF_PTR_SET_LOCK(uintptr_t p) {
    return  ((p) | REF_PTR_LOCK);
  }
  static inline uintptr_t REF_PTR_CLR_LOCK(uintptr_t p) {
    return  ((p) & REF_PTR_MASK);
  }
  static inline uintptr_t REF_PTR_IS_LOCKED(uintptr_t p) {
    return  ((p) & REF_PTR_LOCK);
  }

  uintptr_t lock_ptr() const {
    uintptr_t p = *m_ptr_p;

    p = REF_PTR_CLR_LOCK(p);
    while(!atomic_compare_exchange_ptr(p, REF_PTR_SET_LOCK(p))){
      YIELD();
      p = REF_PTR_CLR_LOCK(p);
    }

    return REF_PTR_SET_LOCK(p);
  }

  void unlock_ptr(uintptr_t ptr) const {
    uintptr_t p = *m_ptr_p;
    if(p != ptr || !atomic_compare_exchange_ptr(p, REF_PTR_CLR_LOCK(p))){
      throw ref_ptr_unlock_error(); 
    }
  }

  protected:
  void replace_ptr(referenceable *ptr) const {
    uintptr_t p = lock_ptr();
    
    if(!atomic_compare_exchange_ptr(p, reinterpret_cast<uintptr_t>(ptr))){
      throw ref_ptr_lock_error(); 
    }
    p = REF_PTR_CLR_LOCK(p);

    if(p){
      reinterpret_cast<referenceable*>(p)->put_ref();
    }
  }

  void set_ptr(referenceable *ptr) const {
    if(REF_PTR_IS_LOCKED(reinterpret_cast<uintptr_t>(ptr))){
      throw ref_ptr_lock_error(); 
    }
    if(ptr){
      ptr->get_ref();
    }

    replace_ptr(ptr);
  }

  referenceable *move_ptr() const {
    uintptr_t p = lock_ptr();
    *m_ptr_p = 0;
    return reinterpret_cast<referenceable*>(REF_PTR_CLR_LOCK(p));
  }

  referenceable *get_ptr() const {
    return reinterpret_cast<referenceable*>((*m_ptr_p)&REF_PTR_MASK);
  }

  referenceable *get_ptr_ref() const {
    uintptr_t p = lock_ptr();
    referenceable *ret = reinterpret_cast<referenceable*>(REF_PTR_CLR_LOCK(p));

    if(ret)ret->get_ref();
    unlock_ptr(p);

    return ret;
  }

  bool compare_exchange(referenceable *expected, ref_ptr_base &&newptr) {
    referenceable *nptr = newptr.move_ptr();
    uintptr_t exp = reinterpret_cast<uintptr_t>(expected);

    if(REF_PTR_IS_LOCKED(exp)){
      throw ref_ptr_lock_error(); 
    }
    bool res = atomic_compare_exchange_ptr(exp, reinterpret_cast<uintptr_t>(nptr));
    if(res) {
      if(expected)expected->put_ref();
    }
    else {
      if(nptr)nptr->put_ref();
    }

    return res;
  }

  ref_ptr_base(): m_ptr_p(new std::decay<decltype(*m_ptr_p)>::type(0)) 
  {
  }

  ~ref_ptr_base() {
    set_ptr((uintptr_t)0);
    delete m_ptr_p;
  }
};

}

template<typename C>
struct is_referenceable{
  private:
  template<typename T>
  static constexpr auto check(T*) -> typename std::is_convertible<T*, referenceable*>::type;

  template<typename>
  static constexpr std::false_type check(...);

  typedef decltype(check<C>(0)) type;

  public:
  static constexpr bool value = type::value;
};

template<typename T>
inline constexpr bool is_referenceable_v = is_referenceable<T>::value;

template<typename T, typename = std::enable_if_t<is_referenceable_v<T>>>
using enable_if_referenceable = T;

template<typename T, typename = std::enable_if_t<!is_referenceable_v<T>>>
using enable_if_not_referenceable = T;


template<typename T, typename ...V>
ref_ptr<T> ref_new(V&&...args);

template<class T> class ref;

template<class T>
class ref: public referenceable, public T{
  template<typename U, typename ...V>
  friend ref_ptr<U> ref_new(V&&...args);
  protected:
  template<typename ...V>
  ref(V...args): T(std::forward<V>(args)...) {}
  ~ref() {}
};

template<class T>
class ref<enable_if_referenceable<T>>: public T {
  template<typename U, typename ...V>
  friend ref_ptr<U> ref_new(V&&...args);
  protected:
  template<typename ...V>
  ref(V...args): T(std::forward<V>(args)...) {}
  ~ref() {}
};


template<class T>
class ref_ptr: protected internals::ref_ptr_base {
  template<typename U, typename ...V>
  friend ref_ptr<U> ref_new(V&&...args);

  template<typename U>
  friend class ref_ptr;

  friend T;

  protected:
  ref_ptr(T *p) { 
    set_ptr(p);
  }

  ref_ptr(ref<T> *p) {
    set_ptr(p);
  }

  T *get_ref() const {
    if constexpr(is_referenceable_v<T>) {
      return dynamic_cast<T *>(get_ptr_ref());
    }
    else {
      return dynamic_cast<ref<T> *>(get_ptr_ref());
    }
  }

  public:
  typedef T type;

  ~ref_ptr() {
    replace_ptr(0);
  }

  ref_ptr() noexcept {}

  ref_ptr(nullptr_t const) noexcept : ref_ptr() { } 

  ref_ptr(const ref_ptr &s) {
    replace_ptr(s.get_ptr_ref());
  }

  ref_ptr(ref_ptr &&s) {
    replace_ptr(s.move_ptr());
  }

  template<class C>
  ref_ptr(const ref_ptr<C> &s) {
    if constexpr(is_referenceable_v<T>) {
      T *ptr = s.get_ref();
      replace_ptr(ptr);
    }
    else {
      ref<T> *ptr = s.get_ref();
      replace_ptr(ptr);
    }
  }

  void reset() {
    replace_ptr(0);
  }

  bool compare_exchange(T *expected, ref_ptr<T> &&newptr) {
    return ref_ptr_base::compare_exchange(expected, std::move(newptr));
  }

  operator bool() const {
    return (get_ptr() != nullptr);
  }

  bool operator == (const ref_ptr &s) const {
    return (get_ptr() == s.get_ptr());
  }

  T& operator*() const {
    if constexpr(is_referenceable_v<T>) {
      return *dynamic_cast<T *>(get_ptr());
    }
    else {
      return *dynamic_cast<ref<T> *>(get_ptr());
    }
  }

  T* operator->() const{
    if constexpr(is_referenceable_v<T>) {
      return dynamic_cast<T *>(get_ptr());
    }
    else {
      return dynamic_cast<ref<T> *>(get_ptr());
    }
  }
};


template<typename C>
struct is_ref_ptr{
  private:
  template<typename T>
  static constexpr auto check(T*) -> typename std::is_same<ref_ptr<typename T::type>, T>::type;

  template<typename>
  static constexpr std::false_type check(...);

  typedef decltype(check<C>(0)) type;

  public:
  static constexpr bool value = type::value;
};


template<typename C>
inline constexpr bool is_ref_ptr_v = is_ref_ptr<C>::value;


template<typename T, typename ...V>
ref_ptr<T> ref_new(V&&...args) {
  if constexpr(is_referenceable_v<T>) {
    return ref_ptr<T>(new T(std::forward<V>(args)...));
  }
  else {
    return ref_ptr<T>(new ref<T>(std::forward<V>(args)...));
  }
}


}

#endif
