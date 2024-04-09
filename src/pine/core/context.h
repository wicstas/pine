#pragma once
#include <pine/core/log.h>

#include <psl/unordered_map.h>
#include <psl/type_traits.h>
#include <psl/function.h>
#include <psl/optional.h>
#include <psl/variant.h>
#include <psl/system.h>
#include <psl/memory.h>
#include <psl/span.h>
#include <psl/map.h>

namespace pine {

struct TypeTag {
  TypeTag() = default;
  TypeTag(psl::string name, bool is_ref = false) : name(psl::move(name)), is_ref(is_ref) {
  }

  psl::string sig() const {
    return name + (is_ref ? "&" : "");
  }

  friend bool operator==(const TypeTag& lhs, const TypeTag& rhs) {
    return lhs.name == rhs.name && lhs.is_ref == rhs.is_ref;
  }

  psl::string name;
  bool is_ref = false;
};
inline psl::string to_string(const TypeTag& type) {
  return type.name + (type.is_ref ? "&" : "");
}

template <typename F, typename R, typename... Args>
struct Lambda {
  Lambda(F lambda) : lambda(psl::move(lambda)) {
  }
  F lambda;
};
template <typename R, typename... Args, typename F>
auto tag(F lambda) {
  return Lambda<F, R, Args...>(psl::move(lambda));
}
template <typename... Args, typename F>
auto lambda(F lambda) {
  return Lambda<F, psl::ReturnType<F, Args...>, Args...>(psl::move(lambda));
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
auto overloaded_const(R (T::*f)(Args...) const) {
  return f;
}
template <typename... Args, typename R>
auto overloaded(R (*f)(Args...)) {
  return f;
}
template <typename R, typename... Args>
auto overloaded_r(R (*f)(Args...)) {
  return f;
}

struct VariableConcept {
  virtual ~VariableConcept() = default;
  virtual psl::unique_ptr<VariableConcept> clone() = 0;
  virtual psl::unique_ptr<VariableConcept> copy() = 0;
  virtual psl::unique_ptr<VariableConcept> create_ref() = 0;
  virtual void* ptr() = 0;
  virtual size_t type_id() const = 0;
  virtual const psl::string& type_name() const = 0;
};

struct Variable {
  template <typename R, typename T>
  struct VariableModel : VariableConcept {
    VariableModel(T base) : base{psl::move(base)} {
    }
    psl::unique_ptr<VariableConcept> clone() override {
      return psl::make_unique<VariableModel>(base);
    }
    psl::unique_ptr<VariableConcept> copy() override {
      if constexpr (psl::is_psl_ref<T>)
        return psl::make_unique<VariableModel<R, R>>(*base);
      else
        return psl::make_unique<VariableModel<R, R>>(base);
    }
    psl::unique_ptr<VariableConcept> create_ref() override {
      if constexpr (psl::is_psl_ref<T>)
        return psl::make_unique<VariableModel<R, T>>(base);
      else
        return psl::make_unique<VariableModel<R, psl::Ref<T>>>(psl::ref(base));
    }
    void* ptr() override {
      if constexpr (psl::is_psl_ref<T>)
        return &(*base);
      else
        return &base;
    }
    size_t type_id() const override {
      return psl::type_id<R>();
    }
    const psl::string& type_name() const override {
      return psl::type_name<R>();
    }

  private:
    T base;
  };

  friend struct VariableConcept;

  Variable() : Variable(psl::Any()) {
  }
  template <typename T>
  Variable(T x) : model(psl::make_unique<VariableModel<T, T>>(psl::move(x))) {
  }
  template <typename T>
  Variable(psl::Ref<T> x) : model(psl::make_unique<VariableModel<T, psl::Ref<T>>>(psl::move(x))) {
  }
  Variable(psl::unique_ptr<VariableConcept> model) : model(psl::move(model)) {
  }
  Variable(const Variable& rhs) : model(rhs.model->clone()) {
  }
  Variable& operator=(const Variable& rhs) {
    model = rhs.model->clone();
    return *this;
  }
  Variable(Variable&& rhs) = default;
  Variable& operator=(Variable&& rhs) = default;
  Variable(Variable*) = delete;
  Variable(const Variable*) = delete;
  Variable& operator=(Variable*) = delete;
  Variable& operator=(const Variable*) = delete;

  Variable copy() const {
    return Variable(model->copy());
  }
  Variable create_ref() const {
    return Variable(model->create_ref());
  }
  size_t type_id() const {
    return model->type_id();
  }
  const psl::string& type_name() const {
    return model->type_name();
  }
  template <typename T>
  bool is() const {
    return model->type_id() == psl::type_id<T>();
  }
  template <typename T>
  T as() const {
    using Base = psl::Decay<T>;
    // #ifndef NDEBUG
    if (!is<Base>())
      Fatal("Trying to interpret ", type_name(), " as ", psl::type_name<T>());
    // #endif
    return *reinterpret_cast<Base*>(model->ptr());
  }

  void* ptr() const {
    return model->ptr();
  }

private:
  psl::unique_ptr<VariableConcept> model;
};

inline psl::string signature_from(const TypeTag& rtype, psl::span<const TypeTag> ptypes) {
  auto r = psl::string("(");
  for (const auto& ptype : ptypes)
    r += ptype.sig() + ", ";
  if (ptypes.size() != 0)
    r.pop_back(2);
  r += "): " + rtype.sig();
  return r;
}

struct Function {
  struct FunctionConcept {
    virtual ~FunctionConcept() = default;

    virtual Variable call(psl::span<const Variable*> args) = 0;
    virtual TypeTag rtype() const = 0;
    virtual psl::span<const TypeTag> ptypes() const = 0;
  };

  template <typename Repr, typename F>
  struct FunctionModel;
  template <typename Repr, typename R, typename... Args>
  struct FunctionModel<Repr, R(Args...)> : FunctionConcept {
    static_assert(sizeof...(Args) <= 8, "Function can only have up to 8 parameters");

    FunctionModel(Repr f, TypeTag rtype, psl::vector<TypeTag> ptypes)
        : f(psl::move(f)), rtype_(psl::move(rtype)), ptypes_(psl::move(ptypes)) {
    }

    Variable call(psl::span<const Variable*> args) override {
      if constexpr (sizeof...(Args) == 1 &&
                    psl::same_as<psl::FirstType<Args..., void>, psl::span<const Variable*>>) {
        if constexpr (psl::SameAs<R, void>)
          return (f(args), Variable());
        else
          return f(args);
      }
      DCHECK_EQ(sizeof...(Args), args.size());
      return [&, this]<typename... Ps>(psl::IndexedTypeSequence<Ps...>) {
        const auto cast = []<typename P>(const Variable& var) -> decltype(auto) {
          if constexpr (psl::is_psl_function<typename P::Type>)
            return (typename P::Type)([f = var.template as<Function>()](auto&&... args) {
              if constexpr (psl::is_void<typename P::Type::ReturnType>)
                f(FWD(args)...);
              else
                return f(FWD(args)...).template as<typename P::Type::ReturnType>();
            });
          else
            return var.template as<typename P::Type>();
        };
        if constexpr (psl::SameAs<R, void>)
          return (f(cast.template operator()<Ps>(*args[Ps::index])...), Variable());
        else
          return f(cast.template operator()<Ps>(*args[Ps::index])...);
      }(psl::make_indexed_type_sequence<Args...>());
    }
    TypeTag rtype() const override {
      return rtype_;
    }
    psl::span<const TypeTag> ptypes() const override {
      return ptypes_;
    }

  private:
    Repr f;
    TypeTag rtype_;
    psl::vector<TypeTag> ptypes_;
  };

  template <typename R, typename... Args>
  Function(R (*f)(Args...), TypeTag rtype, psl::vector<TypeTag> ptypes)
      : model(psl::make_shared<FunctionModel<R (*)(Args...), R(Args...)>>(f, psl::move(rtype),
                                                                          psl::move(ptypes))) {
  }
  template <typename R, typename... Args>
  Function(R& (*f)(Args...), TypeTag rtype, psl::vector<TypeTag> ptypes) {
    auto lambda = [f](Args... args) { return psl::ref(f(static_cast<Args&&>(args)...)); };
    model = psl::make_shared<FunctionModel<decltype(lambda), psl::Ref<R>(Args...)>>(
        lambda, psl::move(rtype), psl::move(ptypes));
  }
  template <typename T, typename R, typename... Args>
  Function(Lambda<T, R, Args...> f, TypeTag rtype, psl::vector<TypeTag> ptypes)
      : model(psl::make_shared<FunctionModel<T, R(Args...)>>(psl::move(f.lambda), psl::move(rtype),
                                                             psl::move(ptypes))) {
  }
  template <typename T, typename R, typename... Args>
  Function(Lambda<T, R&, Args...> f, TypeTag rtype, psl::vector<TypeTag> ptypes) {
    auto lambda = [f = psl::move(f.lambda)](Args... args) {
      return psl::ref(f(static_cast<Args&&>(args)...));
    };
    model = psl::make_shared<FunctionModel<decltype(lambda), psl::Ref<R>(Args...)>>(
        lambda, psl::move(rtype), psl::move(ptypes));
  }

  Variable call(psl::span<const Variable*> args) const {
    return model->call(args);
  }
  Variable operator()(auto&&... args) const {
    Variable args_[]{Variable(FWD(args))...};
    const Variable* args_ptr[sizeof...(args)];
    for (size_t i = 0; i < sizeof...(args); i++)
      args_ptr[i] = &args_[i];
    return call(args_ptr);
  }
  size_t parameter_count() const {
    return ptypes().size();
  }
  TypeTag rtype() const {
    return model->rtype();
  }
  psl::span<const TypeTag> ptypes() const {
    return model->ptypes();
  }
  psl::string signature() const {
    return signature_from(rtype(), ptypes());
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
    Proxy(Context& ctx, psl::string name) : ctx(ctx), name(psl::move(name)) {
    }
    void operator=(auto x) const {
      add(psl::move(x));
    }

  private:
    template <typename F, typename... Ts>
    void add(Overloads<F, Ts...> f) const {
      (add(pine::tag<psl::ReturnType<F, Ts>, Ts>(f.f)), ...);
    }
    template <typename F, typename... Ts, typename... Us>
    void add(OverloadsSet<F, Overloads<Ts...>, Overloads<Us...>> f) const {
      (
          [&]<typename U>() {
            (add(pine::tag<psl::ReturnType<F, Ts, U>, Ts, U>(f.f)), ...);
          }.template operator()<Us>(),
          ...);
    }
    template <typename F>
    requires psl::is_function<F>
    void add(F f) const {
      ctx.add_f(psl::move(name), f);
    }
    template <typename F, typename R, typename... Args>
    void add(Lambda<F, R, Args...> lambda) const {
      ctx.add_f(psl::move(name), psl::move(lambda));
    }
    void add(Function f) const {
      ctx.add_f(psl::move(name), psl::move(f));
    }
    template <typename T>
    void add(T value) const {
      ctx.add_variable(psl::move(name), psl::move(value));
    }

    Context& ctx;
    psl::string name;
  };

  struct TypeTrait {
    TypeTrait() = default;
    TypeTrait(psl::string type_name) : type_name_(psl::move(type_name)) {
    }
    void add_member_accessor(psl::string member_name, size_t index) {
      member_accessors[member_name] = index;
    }
    void add_from_converter(psl::string from_type, size_t index) {
      from_converters[from_type] = index;
    }
    size_t find_member_accessor_index(psl::string_view member_name) const {
      return psl::find_or(member_accessors, member_name, size_t(-1));
    }
    size_t find_from_converter_index(psl::string_view from_type) const {
      return psl::find_or(from_converters, from_type, size_t(-1));
    }
    const psl::string& type_name() const {
      return type_name_;
    }

  private:
    psl::string type_name_;
    psl::map<psl::string, size_t> member_accessors;
    psl::map<psl::string, size_t> from_converters;
  };

  enum TypeClass { None, Float, Complex };

  template <typename T>
  struct TypeProxy {
    TypeProxy(Context& ctx, TypeTrait& trait) : ctx(ctx), trait(trait) {
    }
    template <typename U>
    TypeProxy& member(psl::string name, U T::*ptr) {
      trait.add_member_accessor(name, ctx.functions.size());
      ctx.add_f(pine::tag<U&, T&>([ptr](T& x) -> U& { return x.*ptr; }));
      return *this;
    }
    template <typename U, typename R, typename... Args>
    TypeProxy& method(psl::string name, R (U::*f)(Args...)) {
      ctx(name) = derived<T>(f);
      return *this;
    }
    template <typename U, typename R, typename... Args>
    TypeProxy& method(psl::string name, R (U::*f)(Args...) const) {
      ctx(name) = derived<T>(f);
      return *this;
    }
    template <typename F>
    TypeProxy& method(psl::string name, F f) {
      ctx(name) = std::move(f);
      return *this;
    }
    template <typename... Args>
    TypeProxy& ctor() {
      ctx.add_f(
          trait.type_name(), +[](Args... args) { return T(FWD(args)...); });
      return *this;
    }
    template <typename F>
    TypeProxy& ctor(F f) {
      ctx.add_f(trait.type_name(), psl::move(f));
      return *this;
    }
    template <typename... Args>
    TypeProxy& ctor_variant_explicit() {
      (ctx.add_f(
           trait.type_name(), +[](Args arg) { return T(static_cast<Args&&>(arg)); }),
       ...);
      return *this;
    }
    template <typename... Args>
    TypeProxy& ctor_variant() {
      [&]<typename... Ts>(psl::IndexedTypeSequence<Ts...>) {
        (trait.add_from_converter(ctx.tag<typename Ts::Type>().name,
                                  ctx.functions.size() + Ts::index),
         ...);
        (ctx.add_f(
             trait.type_name(),
             +[](typename Ts::Type arg) { return T(static_cast<typename Ts::Type&&>(arg)); }),
         ...);
        (ctx.add_f(
             "=",
             +[](T& x, typename Ts::Type arg) -> T& {
               return x = T(static_cast<typename Ts::Type&&>(arg));
             }),
         ...);
      }(psl::make_indexed_type_sequence<Args...>());

      return *this;
    }
    template <typename... Args>
    TypeProxy& converter(auto f) {
      [&]<typename... Ts>(psl::IndexedTypeSequence<Ts...>) {
        (trait.add_from_converter(ctx.tag<typename Ts::Type>().name,
                                  ctx.functions.size() + Ts::index),
         ...);
        (ctx.add_f(trait.type_name(), pine::tag<T, Args>(f)), ...);
      }(psl::make_indexed_type_sequence<Args...>());

      return *this;
    }

  private:
    Context& ctx;
    TypeTrait& trait;
  };
  template <typename T>
  requires(!std::is_class_v<T>)
  struct TypeProxy<T> {
    TypeProxy(Context& ctx, TypeTrait& trait) : ctx(ctx), trait(trait) {
    }
    template <typename... Args>
    TypeProxy& ctor_variant_explicit() {
      (ctx.add_f(
           trait.type_name(), +[](Args arg) { return T(static_cast<Args&&>(arg)); }),
       ...);
      return *this;
    }
    template <typename... Args>
    TypeProxy& ctor_variant() {
      [&]<typename... Ts>(psl::IndexedTypeSequence<Ts...>) {
        (trait.add_from_converter(ctx.tag<typename Ts::Type>().name,
                                  ctx.functions.size() + Ts::index),
         ...);
        (ctx.add_f(
             trait.type_name(),
             +[](typename Ts::Type arg) { return T(static_cast<typename Ts::Type&&>(arg)); }),
         ...);
        (ctx.add_f(
             "=",
             +[](T& x, typename Ts::Type arg) -> T& {
               return x = T(static_cast<typename Ts::Type&&>(arg));
             }),
         ...);
      }(psl::make_indexed_type_sequence<Args...>());

      return *this;
    }

  private:
    Context& ctx;
    TypeTrait& trait;
  };

  Context();

  Proxy operator()(psl::string name) {
    return Proxy(*this, psl::move(name));
  }
  template <typename T>
  auto type() {
    return TypeProxy<T>(*this, get_type_trait<T>());
  }
  template <typename T, TypeClass type_class = TypeClass::None>
  auto type(psl::string alias);

  template <typename F>
  void add_f(psl::string name, F f) {
    auto func = wrap(f);
    auto [first, last] = functions_map.equal_range(name);
    while (first != last) {
      const auto& [fname, fi] = *first;
      if (functions[fi].signature() == func.signature()) {
        functions_map.erase(first);
        std::tie(first, last) = functions_map.equal_range(name);
      } else {
        ++first;
      }
    }
    functions_map.insert({psl::move(name), functions.size()});
    add_f(psl::move(func));
  }
  template <typename F>
  void add_f(F f) {
    add_f(wrap(psl::move(f)));
  }
  void add_f(Function func);

  struct FindUniqueFResult {
    size_t function_index;
    enum Status { None, TooMany, Found } status;
    psl::string candidates;
  };
  FindUniqueFResult find_unique_f(psl::string_view name) const;

  struct FindFResult {
    size_t function_index;
    TypeTag rtype;
    struct ArgumentConversion {
      size_t position;
      size_t converter_index;
      TypeTag rtype;
    };
    psl::vector<ArgumentConversion> converts;
  };
  FindFResult find_f(psl::string_view name, psl::span<const TypeTag> atypes) const;

  Variable call(psl::string_view name, psl::span<const TypeTag> atypes,
                psl::vector<Variable> args) const;

  // Argument types must match parameter types
  template <typename T, typename... Args>
  T call(psl::string_view name, Args&&... args) const {
    return call(name, tags<Args...>(), psl::vector_of<Variable>(FWD(args)...)).template as<T>();
  }

  uint16_t find_variable(psl::string_view name) const;
  template <typename T>
  void add_variable(psl::string name, T x) {
    variables_map.insert({psl::move(name), variables.size()});
    variables.push_back({tag<T>(), psl::move(x)});
  }

  TypeTrait& get_type_trait(psl::string_view name);
  const TypeTrait& get_type_trait(psl::string_view name) const;

  template <typename T>
  TypeTrait& get_type_trait() {
    return get_type_trait(name_of<T>());
  }
  template <typename T>
  const TypeTrait& get_type_trait() const {
    return get_type_trait(name_of<T>());
  }
  psl::string type_name_from_id(size_t type_id) const {
    if (auto it = types_map.find(type_id); it != types_map.end())
      return it->second;
    else
      Fatal("Type with id `", type_id, "` is not registered");
  }
  template <typename T>
  psl::string name_of() const {
    if constexpr (psl::is_psl_function<T>)
      return name_of_function_type(reinterpret_cast<T*>(0));
    if (auto it = types_map.find(psl::type_id<T>()); it != types_map.end())
      return it->second;
    else
      Fatal("Type `", psl::type_name<T>(), "` is not registered");
  }
  template <typename R, typename... Args>
  psl::string name_of_function_type(psl::function<R(Args...)>*) const {
    return signature_from(tag<R>(), tags<Args...>());
  }
  bool is_registered_type(psl::string_view name) const {
    return types.find(name) != types.end();
  }
  template <typename T>
  TypeTag tag() const {
    return TypeTag(name_of<T>(), psl::is_reference<T> && !psl::is_const_ref<T>);
  }
  template <typename... Ts>
  psl::vector<TypeTag> tags() const {
    return psl::vector_of<TypeTag>(tag<Ts>()...);
  }

  size_t next_internal_class_id() const {
    return internal_class_n;
  }
  size_t new_internal_class() {
    return internal_class_n++;
  }

  psl::string complete(psl::string part) const;

  psl::vector<Function> functions;
  psl::multimap<psl::string, size_t> functions_map;
  psl::vector<psl::pair<TypeTag, Variable>> variables;
  psl::multimap<psl::string, size_t> variables_map;
  psl::unordered_map<size_t, psl::string> types_map;
  psl::map<psl::string, TypeTrait> types;
  size_t internal_class_n = 0;
  psl::optional<TypeTag> function_rtype;

private:
  Function wrap(Function f) const {
    return f;
  }
  template <typename R, typename... Args>
  Function wrap(R (*f)(Args...)) const {
    return Function(f, tag<R>(), psl::vector_of<TypeTag>(tag<Args>()...));
  }
  template <typename T, typename R, typename... Args>
  Function wrap(R (T::*f)(Args...)) const {
    auto lambda = pine::tag<R, T&, Args...>(
        [f](T& x, Args... args) -> R { return (x.*f)(static_cast<Args&&>(args)...); });
    return Function(psl::move(lambda), tag<R>(), psl::vector_of(tag<T&>(), tag<Args>()...));
  }
  template <typename T, typename R, typename... Args>
  Function wrap(R (T::*f)(Args...) const) const {
    auto lambda = pine::tag<R, const T&, Args...>(
        [f](const T& x, Args... args) -> R { return (x.*f)(static_cast<Args&&>(args)...); });
    return Function(psl::move(lambda), tag<R>(), psl::vector_of(tag<const T&>(), tag<Args>()...));
  }
  template <typename T, typename R, typename... Args>
  Function wrap(Lambda<T, R, Args...> f) const {
    return Function(psl::move(f), tag<R>(), psl::vector_of<TypeTag>(tag<Args>()...));
  }
};

template <typename T, Context::TypeClass type_class>
auto Context::type(psl::string alias) {
  types_map[psl::type_id<T>()] = alias;
  auto& trait = types[alias] = TypeTrait(alias);
  // clang-format off
  if constexpr (type_class == TypeClass::Float || type_class == TypeClass::Complex) {
    add_f("+", +[](T a, T b) { return a + b; });
    add_f("-", +[](T a, T b) { return a - b; });
    add_f("*", +[](T a, T b) { return a * b; });
    add_f("/", +[](T a, T b) { return a / b; });
    add_f("+=", +[](T& a, T b) -> T& { return a += b; });
    add_f("-=", +[](T& a, T b) -> T& { return a -= b; });
    add_f("*=", +[](T& a, T b) -> T& { return a *= b; });
    add_f("/=", +[](T& a, T b) -> T& { return a /= b; });
    add_f("-x", +[](T x) -> T { return -x; });
    add_f("==", +[](T a, T b) -> bool { return a == b; });
    add_f("!=", +[](T a, T b) -> bool { return a != b; });
    add_f("=", +[](T& a, T b) -> T& { return a = b; });
    if constexpr (type_class == TypeClass::Float) {
      add_f("<", +[](T a, T b) -> bool { return a < b; });
      add_f(">", +[](T a, T b) -> bool { return a > b; });
      add_f("<=", +[](T a, T b) -> bool { return a <= b; });
      add_f(">=", +[](T a, T b) -> bool { return a >= b; });
    }
  }
  // clang-format on
  return TypeProxy<T>(*this, trait);
}

}  // namespace pine