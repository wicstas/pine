#pragma once

#include <pine/core/log.h>

#include <pine/psl/optional.h>
#include <pine/psl/system.h>
#include <pine/psl/memory.h>
#include <pine/psl/string.h>
#include <pine/psl/map.h>

namespace pine {

template <typename T, typename R, typename... Args>
struct Lambda {
  Lambda(T lambda) : lambda{psl::move(lambda)} {
  }
  T lambda;
};
template <typename R, typename... Args>
auto tag(auto lambda) {
  return Lambda<decltype(lambda), R, Args...>{psl::move(lambda)};
}
template <typename T, typename U, typename R, typename... Args>
auto derived(R (U::*f)(Args...)) {
  return static_cast<R (T::*)(Args...)>(f);
}
template <typename T, typename U, typename R, typename... Args>
auto derived(R (U::*f)(Args...) const) {
  return static_cast<R (T::*)(Args...) const>(f);
}
template <typename R, typename... Args, typename T>
auto overload(R (T::*f)(Args...)) {
  return f;
}
template <typename R, typename... Args>
auto overload(R (*f)(Args...)) {
  return f;
}

struct Variable;
struct VariableConcept;

struct TypeConcept {
  virtual ~TypeConcept() = default;
  struct MemberVarConcept {
    virtual ~MemberVarConcept() = default;
    virtual Variable get_var(void* x) const = 0;
  };
  struct ConverterConcept {
    virtual ~ConverterConcept() = default;
    virtual Variable convert(const void* x) const = 0;
  };

  const MemberVarConcept* find_var_concept(psl::string_view name) const {
    if (auto it = var_concepts.find(name); it != var_concepts.end())
      return it->second.get();
    else
      return nullptr;
  }
  const ConverterConcept* find_converter(size_t type_id) const {
    if (auto it = converters.find(type_id); it != converters.end())
      return it->second.get();
    else
      return nullptr;
  }

  virtual size_t id() const = 0;
  virtual const psl::string& name() const = 0;
  const psl::string& alias() const {
    if (alias_ == "")
      Fatal("The alias of `", name(), "` is not specified");
    return alias_;
  }

  void set_alias(psl::string str) {
    if (str == "")
      Fatal("Can't set the empty as alias");
    alias_ = psl::move(str);
  }

protected:
  psl::map<psl::string, psl::unique_ptr<MemberVarConcept>> var_concepts;
  psl::map<size_t, psl::unique_ptr<ConverterConcept>> converters;
  psl::string alias_;
};

extern psl::map<size_t, psl::shared_ptr<TypeConcept>> types;

template <typename T>
psl::shared_ptr<TypeConcept>& var_type_ptr();
template <typename T>
TypeConcept& var_type() {
  return *var_type_ptr<T>();
}

template <typename T>
auto var_type_alias() {
  constexpr auto prefix = psl::IsConst<T> ? "const " : "";
  constexpr auto suffix = psl::IsReference<T> ? "&" : "";
  return prefix + var_type<T>().alias() + suffix;
}

struct VariableConcept {
  virtual ~VariableConcept() = default;
  virtual psl::shared_ptr<VariableConcept> clone() const = 0;
  virtual void* ptr() = 0;
  virtual const void* ptr() const = 0;
  virtual Variable member_var(psl::string_view name) = 0;

  virtual psl::optional<Variable> convert(size_t type_id) const = 0;
  virtual bool convertible_to(size_t type_id) const = 0;

  virtual size_t type_id() const = 0;
  virtual psl::string type_name() const = 0;
  virtual psl::string type_alias() const = 0;
};

struct FunctionConcept {
  virtual ~FunctionConcept() = default;
  virtual psl::shared_ptr<FunctionConcept> clone() const = 0;

  virtual Variable call(const psl::vector<Variable>& args) = 0;
  virtual const psl::vector<psl::TypeId>& arg_type_ids() const = 0;
  virtual psl::vector<psl::string> arg_type_aliases() const = 0;
  virtual bool is_internal_function() const = 0;
};

struct Variable {
  template <typename R, typename T>
  struct VariableModel : VariableConcept {
    VariableModel(T base) : base{psl::move(base)} {};

    psl::shared_ptr<VariableConcept> clone() const override {
      return psl::make_shared<VariableModel>(*this);
    }
    void* ptr() override {
      return reinterpret_cast<void*>(&static_cast<R&>(base));
    }
    const void* ptr() const override {
      return reinterpret_cast<const void*>(&static_cast<const R&>(base));
    }
    Variable member_var(psl::string_view name) override {
      if constexpr (!psl::FundamentalType<R>) {
        if (auto p = type().find_var_concept(name); p)
          return p->get_var(ptr());
        else
          exception("Unable to find member `", name, "` in type `", type().alias(), '`');
      } else {
        exception("Fundamental type `", type().alias(), "` doesn't have any member");
      }
      PINE_UNREACHABLE;
    }

    psl::optional<Variable> convert(size_t type_id) const override {
      if (auto p = type().find_converter(type_id); p)
        return p->convert(ptr());
      else
        return psl::nullopt;
    }
    bool convertible_to(size_t type_id) const override {
      return type().find_converter(type_id) != nullptr;
    }

    size_t type_id() const override {
      return type().id();
    }
    psl::string type_name() const override {
      return type().name();
    }
    psl::string type_alias() const override {
      return var_type_alias<R>();
    }

  private:
    const TypeConcept& type() const {
      return var_type<R>();
    }

    T base;
  };

  Variable() = default;
  template <typename T>
  Variable(T x)
      : model(psl::make_shared<VariableModel<psl::RemoveReference<T>, T>>(psl::move(x))) {
  }
  Variable make_copy() const {
    CHECK(model);
    auto rhs = Variable{};
    rhs.model = model->clone();
    return rhs;
  }
  Variable operator[](psl::string_view name) const {
    CHECK(model);
    return model->member_var(name);
  }
  size_t type_id() const {
    CHECK(model);
    return model->type_id();
  }
  psl::string type_name() const {
    CHECK(model);
    return model->type_name();
  }
  psl::string type_alias() const {
    CHECK(model);
    return model->type_alias();
  }
  template <typename T>
  bool is() const {
    CHECK(model);
    return model->type_id() == psl::type_id<T>();
  }
  template <typename T>
  T as() const {
    CHECK(model);
    using Base = psl::RemoveConst<psl::RemoveReference<T>>;
    if (!is<Base>()) {
      if (psl::IsReference<T> || psl::IsRvReference<T>)
        exception("Unable to convert `", model->type_alias(), "` to a reference type `",
                  var_type_alias<T>(), '`');
      if (auto r = model->convert(psl::type_id<T>()); r)
        return r->template as<T>();
      exception("Unable to convert `", model->type_alias(), "` to `", var_type_alias<T>(), '`');
    }
    return *reinterpret_cast<Base*>(model->ptr());
  }
  bool convertible_to(size_t type_id) const {
    CHECK(model);
    return model->convertible_to(type_id);
  }
  bool exist() const {
    return bool(model);
  }

private:
  psl::shared_ptr<VariableConcept> model;
};

struct Function {
  template <typename T, typename R, typename... Args>
  struct FunctionModel : FunctionConcept {
    FunctionModel(T base) : base{psl::move(base)} {};

    psl::shared_ptr<FunctionConcept> clone() const override {
      return psl::make_shared<FunctionModel>(*this);
    }

    Variable call(const psl::vector<Variable>& args) override {
      if constexpr (sizeof...(Args) == 1 && psl::SameAs<psl::Decay<psl::FirstType<Args..., void>>,
                                                        psl::vector<Variable>>) {
        if constexpr (psl::SameAs<R, void>)
          return (base(args), Variable{});
        else
          return base(args);
      } else {
        return [&args, this]<typename... Ps>(psl::IndexedTypeSequence<Ps...>) {
          auto cast = []<typename P>(const Variable& var) -> decltype(auto) {
            static_assert(!psl::SameAs<typename P::Type, Variable&> &&
                              !psl::SameAs<typename P::Type, Variable&&>,
                          "");
            if constexpr (psl::SameAs<typename P::Type, Variable> ||
                          psl::SameAs<typename P::Type, const Variable&>)
              return var;
            else
              return var.template as<typename P::Type>();
          };
          if constexpr (psl::SameAs<R, void>)
            return (base(cast.template operator()<Ps>(args[Ps::index])...), Variable{});
          else
            return Variable{base(cast.template operator()<Ps>(args[Ps::index])...)};
        }(psl::make_indexed_type_sequence<Args...>());
      }
    }
    const psl::vector<psl::TypeId>& arg_type_ids() const override {
      static auto arg_type_ids_ = psl::vector_of<psl::TypeId>(psl::type_id_full<Args>()...);
      return arg_type_ids_;
    }
    psl::vector<psl::string> arg_type_aliases() const override {
      return psl::vector_of<psl::string>(var_type_alias<Args>()...);
    }
    bool is_internal_function() const override {
      if constexpr (sizeof...(Args) == 1)
        if constexpr (psl::SameAs<psl::Decay<psl::FirstType<Args...>>, psl::vector<Variable>>)
          return true;
      return false;
    }

  private:
    T base;
  };
  template <typename R, typename... Args>
  FunctionModel(R (*)(Args...)) -> FunctionModel<R (*)(Args...), R, Args...>;

  Function() = default;
  template <typename R, typename... Args>
  Function(R (*f)(Args...))
      : model(psl::make_shared<FunctionModel<R (*)(Args...), R, Args...>>(f)) {
  }
  template <typename T, typename R, typename... Args>
  Function(R (T::*f)(Args...)) {
    auto lambda = [f](T& x, Args... args) { return (x.*f)(psl::forward<Args>(args)...); };
    model = psl::make_shared<FunctionModel<decltype(lambda), R, T&, Args...>>(psl::move(lambda));
  }
  template <typename T, typename R, typename... Args>
  Function(R (T::*f)(Args...) const) {
    auto lambda = [f](const T& x, Args... args) { return (x.*f)(psl::forward<Args>(args)...); };
    model = psl::make_shared<FunctionModel<decltype(lambda), R, const T&, Args...>>(psl::move(lambda));
  }
  template <typename T, typename R, typename... Args>
  Function(Lambda<T, R, Args...> f)
      : model(psl::make_shared<FunctionModel<T, R, Args...>>(psl::move(f.lambda))) {
  }
  Function make_copy() const {
    CHECK(model);
    auto rhs = Function{};
    rhs.model = model->clone();
    return rhs;
  }
  Variable operator()(const psl::vector<Variable>& args) const {
    CHECK(model);
    return model->call(args);
  }
  const psl::vector<psl::TypeId>& arg_type_ids() const {
    CHECK(model);
    return model->arg_type_ids();
  }
  psl::vector<psl::string> arg_type_aliases() const {
    CHECK(model);
    return model->arg_type_aliases();
  }
  bool is_internal_function() const {
    CHECK(model);
    return model->is_internal_function();
  }
  bool exist() const {
    return bool(model);
  }

private:
  psl::shared_ptr<FunctionConcept> model;
};

template <typename T>
struct TypeModel : TypeConcept {
  using TypeConcept::TypeConcept;

  template <typename U, typename MemberT>
  TypeModel& add_var(const psl::string& name, MemberT U::*ptr) {
    struct MemberVarModel : TypeConcept::MemberVarConcept {
      MemberVarModel(MemberT U::*ptr) : ptr{ptr} {
      }
      Variable get_var(void* x) const override {
        static_assert(psl::IsConvertible<T*, U*>, "");
        if constexpr (psl::IsConvertible<T*, U*>)
          return Variable{psl::ref{reinterpret_cast<T*>(x)->*ptr}};
      }
      MemberT U::*ptr;
    };
    var_concepts[name] = psl::make_unique<MemberVarModel>(ptr);
    return *this;
  }

  template <typename R, typename F>
  requires psl::SameAs<R, psl::ReturnType<F, T&>>
  TypeModel& add_converter(F f) {
    struct ConverterModel : TypeConcept::ConverterConcept {
      ConverterModel(F f) : f{psl::move(f)} {
      }
      Variable convert(const void* x) const override {
        return Variable{f(*reinterpret_cast<const T*>(x))};
      }
      F f;
    };
    converters[psl::type_id<R>()] = psl::make_unique<ConverterModel>(f);
    return *this;
  }

  size_t id() const override {
    return psl::type_id<T>();
  }
  const psl::string& name() const override {
    return psl::type_name<T>();
  }
};

template <typename T>
psl::shared_ptr<TypeConcept>& var_type_ptr() {
  auto& ptr = types[psl::type_id<T>()];
  if (!ptr)
    ptr = psl::make_shared<TypeModel<T>>();
  return ptr;
}

template <typename T, typename... Args>
constexpr auto ctor = +[](Args... args) { return T(psl::move(args)...); };
template <typename T, typename... Args>
const auto ctors = psl::tuple{ctor<T, Args>...};

enum class Class { None, BoolLike, IntLike, FloatLike, ComplexLike };

psl::vector<psl::string> split(psl::string_view input, char key);

struct SourceLines {
  SourceLines() = default;
  SourceLines(psl::string_view tokens, size_t lines_padding);

  psl::optional<psl::string_view> next_line(size_t row) const;

  psl::optional<char> next(size_t row, size_t column) const;

  template <typename... Args>
  void error(size_t row, size_t column, const Args&... args) const {
    error_impl(row, column, psl::to_string(args...));
  }

  void error_impl(size_t row, size_t column, psl::string_view message) const;

private:
  psl::vector<psl::string> lines;
  size_t lines_padding = invalid;
  static constexpr size_t invalid = static_cast<size_t>(-1);
};

struct Context {
  struct VarProxy {
    template <typename R, typename... Args>
    void add(R (*f)(Args...)) const {
      ctx.functions.insert({name, Function{f}});
    }
    template <typename T, typename R, typename... Args>
    void add(R (T::*f)(Args...)) const {
      ctx.functions.insert({name, Function{f}});
    }
    template <typename T, typename R, typename... Args>
    void add(R (T::*f)(Args...) const) const {
      ctx.functions.insert({name, Function{f}});
    }
    template <typename T, typename R, typename... Args>
    void add(Lambda<T, R, Args...> f) const {
      ctx.functions.insert({name, Function{psl::move(f)}});
    }
    template <typename T>
    void add(T x) const {
      ctx.top()[name] = Variable{psl::move(x)};
    }
    void operator=(auto x) const {
      add(psl::move(x));
    }
    template <typename... Fs>
    void operator=(psl::tuple<Fs...> tp) const {
      psl::apply(psl::move(tp), [&](auto... xs) { (add(psl::move(xs)), ...); });
    }

    Context& ctx;
    psl::string name;
  };
  VarProxy operator()(psl::string name) {
    return VarProxy{*this, name};
  }
  Variable* find(psl::string_view name);
  Variable& operator[](psl::string_view name);
  Function f(psl::string_view name, const psl::vector<Variable>& args);
  template <typename... Args>
  Variable call(psl::string_view name, Args&&... args) {
    auto args_ = psl::vector_of<Variable>(psl::forward<Args>(args)...);
    return f(name, args_)(args_);
  }
  Variable call1(psl::string_view name, const psl::vector<Variable>& args) {
    return f(name, args)(args);
  }

  template <typename T>
  struct TypeProxy {
    template <typename U, typename MemberT>
    TypeProxy& var(const psl::string& name, MemberT U::*ptr) {
      model().add_var(name, ptr);
      return *this;
    }
    template <typename U, typename R, typename... Args>
    TypeProxy& method(const psl::string& name, R (U::*f)(Args...)) {
      ctx(name) = derived<T>(f);
      return *this;
    }
    template <typename U, typename R, typename... Args>
    TypeProxy& method(const psl::string& name, R (U::*f)(Args...) const) {
      ctx(name) = derived<T>(f);
      return *this;
    }
    template <typename R, typename... Args>
    TypeProxy& method(const psl::string& name, R (*f)(Args...)) {
      ctx(name) = f;
      return *this;
    }
    template <typename R, typename... Args, typename F>
    TypeProxy& method(const psl::string& name, F f) {
      ctx(name) = tag<R, Args...>(psl::move(f));
      return *this;
    }
    template <typename R, typename F>
    TypeProxy& to(F f) {
      model().template add_converter<R>(f);
      return *this;
    }
    template <typename R>
    TypeProxy& to() {
      return to<R>(+[](T x) { return R(psl::move(x)); });
    }
    template <typename... Ts>
    TypeProxy& ctor() {
      ctx(var_type<T>().alias()) = pine::ctor<T, Ts...>;
      return *this;
    }
    template <typename... Ts>
    TypeProxy& ctors() {
      ctx(var_type<T>().alias()) = pine::ctors<T, Ts...>;
      (TypeProxy<Ts>(ctx).template to<T>(), ...);
      return *this;
    }

    TypeModel<T>& model() const {
      return static_cast<TypeModel<T>&>(var_type<T>());
    }

    Context& ctx;
  };
  template <typename T, Class class_ = Class::None>
  auto type(psl::string alias) {
    CHECK_NE(alias, "");
    (*this)("=") = +[](T& a, T b) -> decltype(auto) { return a = b; };
    if constexpr (class_ == Class::BoolLike) {
      (*this)("==") = +[](T a, T b) { return a == b; };
      (*this)("!=") = +[](T a, T b) { return a != b; };
      (*this)("||") = +[](T a, T b) { return a || b; };
      (*this)("&&") = +[](T a, T b) { return a && b; };
      (*this)("!x") = +[](T x) { return !x; };
    } else if constexpr (class_ == Class::IntLike) {
      add_arithmetic_ops<T, T>(*this);
      add_comparison_ops<T, T>(*this);
      (*this)("++x") = +[](T& x) -> decltype(auto) { return ++x; };
      (*this)("--x") = +[](T& x) -> decltype(auto) { return --x; };
      (*this)("x++") = +[](T& x) { return x++; };
      (*this)("x--") = +[](T& x) { return x--; };
      (*this)("-x") = +[](T x) { return -x; };
    } else if constexpr (class_ == Class::FloatLike) {
      add_arithmetic_ops<T, T>(*this);
      add_comparison_ops<T, T>(*this);
      (*this)("-x") = +[](T& x) -> decltype(auto) { return -x; };
    } else if constexpr (class_ == Class::ComplexLike) {
      add_arithmetic_ops<T, T>(*this);
      (*this)("==") = +[](T a, T b) { return a == b; };
      (*this)("!=") = +[](T a, T b) { return a != b; };
      (*this)("-x") = +[](T& x) { return -x; };
    }
    var_type<T>().set_alias(psl::move(alias));
    return TypeProxy<T>{*this};
  }

  psl::map<psl::string, Variable>& top();

  void scope_push() {
    variableStack.push_back({});
  }
  void scope_pop() {
    variableStack.pop_back();
  }

  void set_return_value(Variable var) {
    return_value = psl::move(var);
  }
  Variable consume_return_value() {
    return psl::move(return_value);
  }
  bool returned() const {
    return return_value.exist();
  }
  void set_break_flag() {
    break_flag = true;
  }
  void set_continue_flag() {
    continue_flag = true;
  }
  bool consume_break_flag() {
    auto temp = break_flag;
    break_flag = false;
    return temp;
  }
  bool consume_continue_flag() {
    auto temp = continue_flag;
    continue_flag = false;
    return temp;
  }

  template <typename... Args>
  void error(size_t row, size_t column, const Args&... args) const {
    sl.error(row, column, args...);
  }

  SourceLines sl;

private:
  psl::vector<psl::map<psl::string, Variable>> variableStack{1};
  psl::multimap<psl::string, Function> functions;
  Variable return_value;
  bool break_flag = false;
  bool continue_flag = false;
};

template <typename...>
struct Overloads {};

template <typename... Ts>
struct AddFHelper;

template <typename... Ts>
struct AddFHelper<Overloads<Ts...>> {
  void operator()(Context& ctx, psl::string sym, auto f) {
    ((ctx(sym) = tag<decltype(f(psl::declval<Ts>())), Ts>(f)), ...);
  }
};
template <typename... Ts, typename... Us>
struct AddFHelper<Overloads<Ts...>, Overloads<Us...>> {
  void operator()(Context& ctx, psl::string sym, auto f) {
    const auto unpack_ts = [&]<typename U>() {
      ((ctx(sym) = tag<decltype(f(psl::declval<Ts>(), psl::declval<U>())), Ts, U>(f)), ...);
    };
    ((unpack_ts.template operator()<Us>()), ...);
  }
};
template <typename... Ts>
void add_f(Context& ctx, psl::string sym, auto f) {
  AddFHelper<Ts...>{}(ctx, sym, f);
}

template <typename T, typename U>
void add_arithmetic_ops(Context& ctx) {
  ctx("+") = +[](T a, U b) { return a + b; };
  ctx("-") = +[](T a, U b) { return a - b; };
  ctx("*") = +[](T a, U b) { return a * b; };
  ctx("/") = +[](T a, U b) { return a / b; };
  ctx("+=") = +[](T& a, U b) -> decltype(auto) { return a += b; };
  ctx("-=") = +[](T& a, U b) -> decltype(auto) { return a -= b; };
  ctx("*=") = +[](T& a, U b) -> decltype(auto) { return a *= b; };
  ctx("/=") = +[](T& a, U b) -> decltype(auto) { return a /= b; };
}
template <typename T, typename U>
void add_comparison_ops(Context& ctx) {
  ctx("==") = +[](T a, U b) { return a == b; };
  ctx("!=") = +[](T a, U b) { return a != b; };
  ctx("<") = +[](T a, U b) { return a < b; };
  ctx(">") = +[](T a, U b) { return a > b; };
  ctx("<=") = +[](T a, U b) { return a <= b; };
  ctx(">=") = +[](T a, U b) { return a >= b; };
}

}  // namespace pine