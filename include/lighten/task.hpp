#ifndef __LIGHTEN__TASK__HPP
#define __LIGHTEN__TASK__HPP

#include <utility>

namespace lighten {

class task {
  protected:
  class task_base {
    public:
    virtual void run() = 0;
    virtual ~task_base() = default;
  };

  template<typename F, typename...V>
  class task_wrapper: public task_base {
    F f;
    std::tuple<V...> args;

    template <typename... Args, std::size_t... Is>
    void func(std::tuple<Args...>& tup, std::index_sequence<Is...>)
    {
        f(std::forward<Args>(std::get<Is>(tup))...);
    }

    template <typename... Args>
    void func(std::tuple<Args...>& tup)
    {
        func(tup, std::index_sequence_for<Args...>{});
    }
    public:
    task_wrapper(F &&f, V&&...args):f(std::forward<F>(f)), args(std::forward<V>(args)...){}
    virtual ~task_wrapper() = default;
    void run() override {
      func(args);
    }
  };

  task_base *_task;

  public:
  void run() {
    _task->run();
  }

  template<typename F, typename...V>
  task(F &&f, V&&...args): _task(new task_wrapper<F, V...>(std::forward<F>(f), std::forward<V>(args)...)){}

  task(const task &) = delete;
  task(task &&) = default;

  ~task() {
    delete _task;
  }

  class handler {
    public:
    virtual void add_task(task &&) = 0;
  };
};

}

#endif

