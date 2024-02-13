#pragma once
#include <psl/memory.h>

namespace psl {

template <typename T>
class function;

template <typename R, typename... Args>
class function<R(Args...)> {
  struct FunctionConcept {
    virtual ~FunctionConcept() = default;
    virtual R call(Args... args) = 0;
  };
  template <typename F>
  struct FunctionModel : FunctionConcept {
    FunctionModel(F f) : f(psl::move(f)) {
    }

    R call(Args... args) override {
      return f(static_cast<Args>(args)...);
    }
    F f;
  };
  shared_ptr<FunctionConcept> model;

public:
  template <typename F>
  function(F f) : model(psl::make_shared<FunctionModel<F>>(f)) {
  }
  R operator()(Args... args) {
    return model->call(static_cast<Args>(args)...);
  }
};

template <typename T>
struct _is_psl_function : FalseType {};
template <typename T>
struct _is_psl_function<function<T>> : TrueType {};
template <typename T>
constexpr bool is_psl_function = _is_psl_function<T>::value;

template <typename T>
struct _PslFunctionReturnType;
template <typename R, typename... Args>
struct _PslFunctionReturnType<function<R(Args...)>> {
  using Type = R;
};
template <typename T>
using PslFunctionReturnType = typename _PslFunctionReturnType<T>::Type;

}  // namespace psl
