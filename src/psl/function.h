#pragma once
#include <psl/memory.h>

namespace psl {

template <typename T>
class function;

template <typename R, typename... Args>
class function<R(Args...)> {
  struct FunctionConcept {
    virtual ~FunctionConcept() = default;
    virtual R call(Args... args) const = 0;
  };
  template <typename F>
  struct FunctionModel : FunctionConcept {
    FunctionModel(F f) : f(psl::move(f)) {
    }

    R call(Args... args) const override {
      return f(static_cast<Args>(args)...);
    }
    F f;
  };
  shared_ptr<const FunctionConcept> model;

public:
  using ReturnType = R;
  function() = default;
  template <typename F>
  requires requires(F f, Args... args) { f(args...); }
  function(F f) : model(psl::make_shared<FunctionModel<F>>(MOVE(f))) {
  }
  R operator()(Args... args) const {
    return model->call(static_cast<Args>(args)...);
  }
};

template <typename T>
struct _is_psl_function : FalseType {};
template <typename T>
struct _is_psl_function<function<T>> : TrueType {};
template <typename T>
constexpr bool is_psl_function = _is_psl_function<T>::value;

struct opaque_unique_ptr : unique_ptr<Any, function<void(void*)>> {
  opaque_unique_ptr() = default;
  template <typename T>
  opaque_unique_ptr(T* x) : unique_ptr((Any*)x, [](void* ptr) { delete (T*)ptr; }) {
  }
  template <typename T>
  opaque_unique_ptr(T* x, auto f) : unique_ptr((Any*)x, [f](void* ptr) { f((T*)ptr); }) {
  }
};
struct opaque_shared_ptr : shared_ptr<Any, function<void(void*)>> {
  opaque_shared_ptr() = default;
  template <typename T>
  opaque_shared_ptr(T* x) : shared_ptr((Any*)x, [](void* ptr) { delete (T*)ptr; }) {
  }
  template <typename T>
  opaque_shared_ptr(T* x, auto f) : shared_ptr((Any*)x, [f](void* ptr) { f((T*)ptr); }) {
  }
};

template <typename T>
opaque_unique_ptr make_opaque_unique_ptr(T x, auto f) {
  return opaque_unique_ptr(new T(psl::move(x)), f);
}
template <typename T>
opaque_unique_ptr make_opaque_unique_ptr(T x) {
  return opaque_unique_ptr(new T(psl::move(x)));
}
template <typename T>
opaque_shared_ptr make_opaque_shared_ptr(T x, auto f) {
  return opaque_shared_ptr(new T(psl::move(x)), f);
}
template <typename T>
opaque_shared_ptr make_opaque_shared_ptr(T x) {
  return opaque_shared_ptr(new T(psl::move(x)));
}

}  // namespace psl
