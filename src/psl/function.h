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
  using ReturnType = R;
  function() = default;
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
using unique_ptr_with_custom_deleter = unique_ptr<T, psl::function<void(T*)>>;
template <typename T>
using shared_ptr_with_custom_deleter = shared_ptr<T, psl::function<void(T*)>>;

}  // namespace psl
