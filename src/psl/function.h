#pragma once
#include <psl/memory.h>

namespace psl {

template <typename T, typename... Args>
class function {
  struct FunctionConcept {
    virtual ~FunctionConcept() = default;
    virtual T call(Args... args) = 0;
  };
  template <typename F>
  struct FunctionModel : FunctionConcept {
    FunctionModel(F f) : f(psl::move(f)) {
    }

    T call(Args... args) override {
      return f(static_cast<Args>(args)...);
    }
    F f;
  };
  shared_ptr<FunctionConcept> model;

public:
  template <typename F>
  function(F f) : model(psl::make_shared<FunctionModel<F>>(f)) {
  }
  T operator()(Args... args) {
    return model->call(static_cast<Args>(args)...);
  }
};

template <typename T>
struct _is_psl_function : FalseType {};
template <typename R, typename... Args>
struct _is_psl_function<function<R, Args...>> : TrueType {};
template <typename T>
constexpr bool is_psl_function = _is_psl_function<T>::value;


template <typename T>
struct _PslFunctionReturnType;
template <typename R, typename... Args>
struct _PslFunctionReturnType<function<R, Args...>>  {
  using Type = R;
};
template <typename T>
using PslFunctionReturnType = typename _PslFunctionReturnType<T>::Type;

}  // namespace psl
