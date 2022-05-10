#ifndef __LIGHTEN_UTIL__CALLABLE__HPP
#define __LIGHTEN_UTIL__CALLABLE__HPP

#include <type_traits>
#include <lighten/ref.hpp>

namespace lighten {

template <typename T> class callable;

template<class R, class ...A>
class callable<R (A...)>
{
  class wrapper {
    public:
    virtual R invoke(A&&...args) = 0;
  };
  
  class function_wrapper: public wrapper {
    R (*func)(A...args);

    public:
    R invoke(A&&...args) override {
      if constexpr(std::is_void_v<R>) {
        func(std::forward<A>(args)...);
      }
      else {
        return func(std::forward<A>(args)...);
      }
    }
    function_wrapper(R(*f)(A...args)):func(f) {};
  };

  template<class F>
  class functor_wrapper: public wrapper {
    F func;
    public:
    R invoke(A&&...args) override {
      if constexpr(std::is_void_v<R>) {
        func(std::forward<A>(args)...);
      }
      else {
        return func(std::forward<A>(args)...);
      }
    }
    functor_wrapper(const F &f):func(f) {};
    functor_wrapper(F &&f):func(std::forward<F>(f)) {};
  };

  template<typename OBJ_PTR, typename F>
  class member_wrapper: public wrapper {
    OBJ_PTR obj_ptr;
    F func;
    public:
    R invoke(A&&...args) override {
      if constexpr(std::is_void_v<R>) {
        ((*obj_ptr).*func)(std::forward<A>(args)...);
      }
      else {
        return ((*obj_ptr).*func)(std::forward<A>(args)...);
      }
    }
    member_wrapper(const OBJ_PTR &obj_ptr, const F &func):obj_ptr(obj_ptr), func(func) {};
    member_wrapper(OBJ_PTR &&obj_ptr, const F &func):obj_ptr(std::forward(obj_ptr)), func(func) {};
  };

  ref_ptr<wrapper> stub;

  public:

  callable() = default;

  callable(callable const&) = default;

  callable(callable&&) = default;

  callable(std::nullptr_t const) noexcept : callable() { }
  
  callable(R(*f)(A...args)):stub(ref_new<function_wrapper>(f)){}

  template<class F, typename = typename 
    std::enable_if<
      !std::is_same_v<callable, typename std::decay<F>::type>
      &&
      std::is_same_v<
        std::result_of_t<F(A...)>,
        R
      >
    >::type
  >
  callable(const F &f): stub(ref_new<functor_wrapper<F>>(f)) {}

  template<class F, typename = typename 
    std::enable_if<
      !std::is_same_v<
        callable, 
        typename std::decay<F>::type
      >
      &&
      std::is_same_v<
        std::result_of_t<F(A...)>,
        R  
      >
    >::type
  >
  callable(F &&f): stub(ref_new<functor_wrapper<F>>(std::forward<F>(f))) {
  //  stub = ref_new<functor_wrapper<F>>(std::forward<F>(f));
    }

  template<typename OBJ_PTR, typename F, typename = typename std::enable_if<std::is_same_v<std::result_of_t<F(A...)>, R>>::type >
  callable(const OBJ_PTR &obj_ptr, const F &func): stub(ref_new<member_wrapper<OBJ_PTR, F>>(obj_ptr, func)) {}

  template<typename OBJ_PTR, typename F, typename = typename std::enable_if<std::is_same_v<std::result_of_t<F(A...)>, R>>::type >
  callable(OBJ_PTR &&obj_ptr, const F &func): stub(ref_new<member_wrapper<OBJ_PTR, F>>(std::forward<OBJ_PTR>(obj_ptr), func)) {}

  callable &operator = (const callable &s) {
    stub = s.stub;
    return *this;
  }

  callable &operator = (callable &&s) {
    stub = std::move(s.stub);
    return *this;
  }

  callable &operator = (R(*f)(A...args)){
    stub = ref_new<function_wrapper>(f);
    return *this;
  }

  template<class F, typename = typename std::enable_if<!std::is_same_v<callable, typename std::decay<F>::type>
    && std::is_same_v<std::result_of_t<F(A...)>, R>>::type >
  callable &operator = (const F &f) {
    stub = ref_new<functor_wrapper<F>>(f);
    return *this;
  }

  template<class F, typename = typename std::enable_if<!std::is_same_v<callable, typename std::decay<F>::type>
    && std::is_same_v<std::result_of_t<F(A...)>, R>>::type >
  callable &operator = (F &&f) {
    stub = ref_new<functor_wrapper<F>>(std::forward<F>(f));
    return *this;
  }

  operator bool () const {
    return stub;
  }

  R operator()(A...args) const {
    if constexpr(std::is_void_v<R>) {
      stub->invoke(std::forward<A>(args)...);
    }
    else {
      return stub->invoke(std::forward<A>(args)...);
    }
  }
};

}


#endif
