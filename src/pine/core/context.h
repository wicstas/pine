#pragma once

#include <pine/core/log.h>

#include <pine/psl/unordered_map.h>
#include <pine/psl/type_traits.h>
#include <pine/psl/system.h>
#include <pine/psl/memory.h>
#include <pine/psl/span.h>
#include <pine/psl/map.h>

namespace pine {

template <typename T, typename R, typename... Args>
struct Lambda {
  Lambda(T lambda) : lambda(psl::move(lambda)) {
  }
  T lambda;
};
template <typename R, typename... Args, typename F>
auto tag(F lambda) {
  return Lambda<F, R, Args...>(psl::move(lambda));
}
template <typename T, typename U, typename R, typename... Args>
auto derived(R (U::*f)(Args...)) {
  return static_cast<R (T::*)(Args...)>(f);
}
template <typename T, typename U, typename R, typename... Args>
auto derived(R (U::*f)(Args...) const) {
  return static_cast<R (T::*)(Args...) const>(f);
}
template <typename... Args, typename R, typename T>
auto overloaded(R (T::*f)(Args...)) {
  return f;
}
template <typename... Args, typename R, typename T>
auto overloaded(R (T::*f)(Args...) const) {
  return f;
}
template <typename... Args, typename R>
auto overloaded(R (*f)(Args...)) {
  return f;
}

struct VariableConcept {
  virtual ~VariableConcept() = default;
  virtual psl::shared_ptr<VariableConcept> clone() const = 0;
  virtual void* ptr() = 0;
  const void* ptr() const {
    return const_cast<VariableConcept*>(this)->ptr();
  }
  virtual size_t type_id() const = 0;
};

struct Variable {
  template <typename R, typename T>
  struct VariableModel : VariableConcept {
    VariableModel(T base) : base{psl::move(base)} {};
    psl::shared_ptr<VariableConcept> clone() const override {
      return psl::make_shared<VariableModel<R, T>>(*this);
    }
    void* ptr() override {
      return reinterpret_cast<void*>(&static_cast<R&>(base));
    }
    size_t type_id() const override {
      return psl::type_id<R>();
    }

  private:
    T base;
  };

  Variable() = default;
  template <typename T>
  Variable(T x) : model(psl::make_shared<VariableModel<T, T>>(psl::move(x))) {
  }
  template <typename T>
  Variable(psl::ref<T> x) : model(psl::make_shared<VariableModel<T, psl::ref<T>>>(psl::move(x))) {
  }
  Variable clone() const {
    DCHECK(model);
    auto copy = Variable();
    copy.model = model->clone();
    return copy;
  }
  size_t type_id() const {
    DCHECK(model);
    return model->type_id();
  }
  template <typename T>
  bool is() const {
    DCHECK(model);
    return model->type_id() == psl::type_id<T>();
  }
  template <typename T>
  T as() const {
    DCHECK(model);
    using Base = psl::Decay<T>;
    DCHECK(is<Base>());
    // Fatal("Trying to interpret ", model->type_name(), " as ", type_name<T>());
    return *reinterpret_cast<Base*>(model->ptr());
  }

private:
  psl::shared_ptr<VariableConcept> model;
};

struct FunctionConcept {
  virtual ~FunctionConcept() = default;

  virtual Variable call(psl::span<const Variable*> args) = 0;
  virtual size_t n_parameters() const = 0;
  virtual psl::span<psl::TypeId> parameter_type_ids() const = 0;
  virtual size_t return_type_id() const = 0;
};

struct Function {
  template <typename T, typename R, typename... Args>
  struct FunctionModel : FunctionConcept {
    FunctionModel(T base) : base(psl::move(base)){};

    Variable call(psl::span<const Variable*> args) override {
      if constexpr (psl::SameAs<psl::FirstType<Args..., void>, psl::span<const Variable*>>) {
        if constexpr (psl::SameAs<R, void>)
          return (base(args), Variable());
        else
          return Variable(base(args));
      } else {
        DCHECK_EQ(sizeof...(Args), args.size());
        return [&, this]<typename... Ps>(psl::IndexedTypeSequence<Ps...>) {
          const auto cast = []<typename P>(const Variable& var) -> decltype(auto) {
            if constexpr (psl::SameAs<P, const Variable&>)
              return var;
            else
              return var.template as<typename P::Type>();
          };
          if constexpr (psl::SameAs<R, void>)
            return (base(cast.template operator()<Ps>(*args[Ps::index])...), Variable());
          else
            return Variable{base(cast.template operator()<Ps>(*args[Ps::index])...)};
        }(psl::make_indexed_type_sequence<Args...>());
      }
    }
    size_t n_parameters() const override {
      return sizeof...(Args);
    }
    psl::span<psl::TypeId> parameter_type_ids() const override {
      if constexpr (sizeof...(Args) == 0) {
        return {};
      } else {
        static psl::TypeId arg_type_ids_[]{psl::type_id_full<Args>()...};
        return arg_type_ids_;
      }
    }
    size_t return_type_id() const override {
      if constexpr (psl::is_psl_ref<R>)
        return psl::type_id<typename R::BaseType>();
      else
        return psl::type_id<R>();
    }

  private:
    T base;
  };
  // template <typename R, typename... Args>
  // FunctionModel(R (*)(Args...)) -> FunctionModel<R (*)(Args...), R, Args...>;

  Function() = default;
  template <typename R, typename... Args>
  Function(R (*f)(Args...))
      : model(psl::make_shared<FunctionModel<R (*)(Args...), R, Args...>>(f)) {
  }
  template <typename T, typename R, typename... Args>
  Function(R (T::*f)(Args...)) {
    auto lambda = [f](T& x, Args... args) { return (x.*f)(psl::forward<Args>(args)...); };
    model = psl::make_shared<FunctionModel<decltype(lambda), R, T&, Args...>>(lambda);
  }
  template <typename T, typename R, typename... Args>
  Function(R (T::*f)(Args...) const) {
    auto lambda = [f](const T& x, Args... args) { return (x.*f)(psl::forward<Args>(args)...); };
    model = psl::make_shared<FunctionModel<decltype(lambda), R, const T&, Args...>>(lambda);
  }
  template <typename T, typename R, typename... Args>
  Function(Lambda<T, R, Args...> f)
      : model(psl::make_shared<FunctionModel<T, R, Args...>>(psl::move(f.lambda))) {
  }
  Variable call(psl::span<const Variable*> args) const {
    DCHECK(model);
    return model->call(args);
  }
  size_t n_parameters() const {
    DCHECK(model);
    return model->n_parameters();
  }
  psl::span<psl::TypeId> parameter_type_ids() const {
    DCHECK(model);
    return model->parameter_type_ids();
  }
  size_t return_type_id() const {
    DCHECK(model);
    return model->return_type_id();
  }

private:
  psl::shared_ptr<FunctionConcept> model;
};

template <typename F, typename... Ts>
struct Overloads {
  F f;
};
template <typename... Ts, typename F>
requires(sizeof...(Ts) > 0)
auto overloads(F f) {
  return Overloads<F, Ts...>(psl::move(f));
}

template <typename F, typename... Ts>
struct OverloadsSet {
  F f;
};
template <typename... Ts, typename F>
auto overloads_set(F f) {
  return OverloadsSet<F, Ts...>(psl::move(f));
}

struct Context {
  struct Proxy {
    template <typename F, typename... Ts>
    void add(Overloads<F, Ts...> f) const {
      (add(tag<decltype(f.f(psl::declval<Ts>())), Ts>(f.f)), ...);
    }
    template <typename F, typename... Ts, typename... Us>
    void add(OverloadsSet<F, Overloads<Ts...>, Overloads<Us...>> f) const {
      (
          [&]<typename U>() {
            (add(tag<decltype(f.f(psl::declval<Ts>(), psl::declval<U>())), Ts, U>(f.f)), ...);
          }.template operator()<Us>(),
          ...);
    }
    template <typename R, typename... Args>
    void add(R (*f)(Args...)) const {
      ctx.add_f(psl::move(name), Function(f));
    }
    template <typename T, typename R, typename... Args>
    void add(R (T::*f)(Args...)) const {
      ctx.add_f(psl::move(name), Function(f));
    }
    template <typename T, typename R, typename... Args>
    void add(R (T::*f)(Args...) const) const {
      ctx.add_f(psl::move(name), Function(f));
    }
    template <typename T, typename R, typename... Args>
    void add(Lambda<T, R, Args...> f) const {
      ctx.add_f(psl::move(name), Function{psl::move(f)});
    }
    template <typename T>
    void add(T value) const {
      ctx.add_variable(psl::move(name), psl::move(value));
    }
    void operator=(auto x) const {
      add(psl::move(x));
    }

    Context& ctx;
    psl::string name;
  };

  struct TypeTrait {
    size_t find_member_accessor_index(psl::string_view member_name) const;
    size_t find_convert_to_index(size_t to_type_id) const;
    size_t find_convert_from_index(size_t from_type_id) const;

    psl::unordered_map<size_t, size_t> convert_tos;
    psl::unordered_map<size_t, size_t> convert_froms;
    psl::map<psl::string, size_t> member_accessors;
    psl::string alias;
  };

  enum TypeClass { None, Float, Complex };

  template <typename T>
  struct TypeProxy {
    template <typename U>
    TypeProxy& member(psl::string name, U T::*ptr) {
      if (type_trait.member_accessors.find(name) != type_trait.member_accessors.end())
        Fatal("Already set `", name, "` as one of the members of `", type_trait.alias, "`");
      type_trait.member_accessors[name] = ctx.functions.size();
      ctx.functions.push_back(tag<psl::ref<U>, T&>([ptr](T& x) { return psl::ref<U>(x.*ptr); }));
      return *this;
    }
    template <typename F>
    TypeProxy& method(psl::string name, F f) {
      ctx.add_f(psl::move(name), Function(std::move(f)));
      return *this;
    }
    template <typename R>
    TypeProxy& to() {
      type_trait.convert_tos[psl::type_id<R>()] = ctx.functions.size();
      ctx.functions.push_back(+[](const T& x) { return R(x); });
      return *this;
    }
    template <typename... Args>
    TypeProxy& ctor() {
      ctx.add_f(
          type_trait.alias, +[](Args... args) { return T(psl::move(args)...); });
      return *this;
    }
    template <typename... Args>
    TypeProxy& ctor_variant(bool explicit_ = false) {
      if (!explicit_)
        [&]<int... I>(psl::IntegerSequence<int, I...>) {
          ((type_trait.convert_froms[psl::type_id<Args>()] = ctx.functions.size() + I), ...);
        }(psl::make_integer_sequence<int, sizeof...(Args)>());
      (ctx.add_f(
           type_trait.alias, +[](Args arg) { return T(psl::move(arg)); }),
       ...);
      return *this;
    }

    Context& ctx;
    TypeTrait& type_trait;
  };
  template <typename T>
  requires(!std::is_class_v<T>)
  struct TypeProxy<T> {
    template <typename R>
    TypeProxy& to() {
      type_trait.convert_tos[psl::type_id<R>()] = ctx.functions.size();
      ctx.functions.push_back(+[](const T& x) { return R(x); });
      return *this;
    }
    Context& ctx;
    TypeTrait& type_trait;
  };

  Context();

  Proxy operator()(psl::string name) {
    return Proxy(*this, psl::move(name));
  }
  template <typename T, TypeClass type_class = TypeClass::None>
  auto type(psl::string alias) {
    types[psl::type_id<T>()].alias = psl::move(alias);
    if constexpr (type_class == TypeClass::Float || type_class == TypeClass::Complex) {
      add_f(
          "+", +[](T a, T b) { return a + b; });
      add_f(
          "-", +[](T a, T b) { return a - b; });
      add_f(
          "*", +[](T a, T b) { return a * b; });
      add_f(
          "/", +[](T a, T b) { return a / b; });
      add_f(
          "+=", +[](T& a, T b) -> T& { return a += b; });
      add_f(
          "-=", +[](T& a, T b) -> T& { return a -= b; });
      add_f(
          "*=", +[](T& a, T b) -> T& { return a *= b; });
      add_f(
          "/=", +[](T& a, T b) -> T& { return a /= b; });
      add_f(
          "-x", +[](T x) -> T { return -x; });
      add_f(
          "==", +[](T a, T b) -> bool { return a == b; });
      add_f(
          "!=", +[](T a, T b) -> bool { return a != b; });
      add_f(
          "=", +[](T& a, T b) -> T& { return a = b; });
      if constexpr (type_class == TypeClass::Float) {
        add_f(
            "<", +[](T a, T b) -> bool { return a < b; });
        add_f(
            ">", +[](T a, T b) -> bool { return a > b; });
        add_f(
            "<=", +[](T a, T b) -> bool { return a <= b; });
        add_f(
            ">=", +[](T a, T b) -> bool { return a >= b; });
      }
    }
    return TypeProxy<T>(*this, types[psl::type_id<T>()]);
  }

  size_t converter_index(size_t from_id, size_t to_id) const;

  struct FindFResult {
    size_t function_index;
    size_t return_type_id;
    struct ArgumentConversion {
      size_t position;
      size_t converter_id;
      size_t to_type_id;
    };
    psl::vector<ArgumentConversion> converts;
  };
  FindFResult find_f(psl::string_view name, psl::span<size_t> arg_type_ids) const;
  void add_f(psl::string name, Function func);
  size_t find_variable(psl::string_view name) const;
  void add_variable(psl::string name, Variable var);

  psl::multimap<psl::string, size_t> functions_map;
  psl::vector<Function> functions;
  psl::multimap<psl::string, size_t> variables_map;
  psl::vector<Variable> variables;
  psl::unordered_map<size_t, TypeTrait> types;
};

}  // namespace pine