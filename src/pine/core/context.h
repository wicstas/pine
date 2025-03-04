#pragma once
#include <pine/core/log.h>

#include <psl/smart_unique_ptr.h>
#include <psl/unordered_map.h>
#include <psl/type_traits.h>
#include <psl/function.h>
#include <psl/optional.h>
#include <psl/variant.h>
#include <psl/system.h>
#include <psl/memory.h>
#include <psl/array.h>
#include <psl/span.h>
#include <psl/map.h>

namespace pine {

inline bool is_function_type(psl::string_view name) {
  return psl::start_with(name, "(");
}

struct TypeTag {
  TypeTag() = default;
  TypeTag(psl::string name) {
    if (name.back() == '&') {
      name.pop_back();
      is_ref = true;
    }
    this->name = MOVE(name);
  }
  TypeTag(psl::string name, bool is_ref, bool is_const)
      : name(MOVE(name)), is_ref(is_ref), is_const(is_const) {
  }

  psl::string sig() const {
    if (!is_const && is_ref)
      return name + "&";
    else
      return name;
  }

  bool is_function_type() const {
    return pine::is_function_type(name);
  }

  friend bool operator==(const TypeTag& lhs, const TypeTag& rhs) {
    return lhs.name == rhs.name && lhs.is_ref == rhs.is_ref && lhs.is_const == rhs.is_const;
  }

  psl::string name;
  bool is_ref = false;
  bool is_const = false;
};
psl::string signature_from(const TypeTag& rtype, psl::span<const TypeTag> ptypes);

struct Function {
  Function(psl::string name, TypeTag rtype, psl::vector<TypeTag> ptypes, void* f)
      : name_(MOVE(name)), rtype_(MOVE(rtype)), ptypes_(MOVE(ptypes)), f(MOVE(f)) {
  }

  TypeTag rtype() const {
    return rtype_;
  }
  psl::span<const TypeTag> ptypes() const {
    return ptypes_;
  }
  const psl::string& name() const {
    return name_;
  }
  void* ptr() const {
    return (void*)f;
  }
  psl::string signature() const {
    return signature_from(rtype(), ptypes());
  }
  psl::string unique_name() const {
    return name() + signature_from(rtype(), ptypes());
  }

  void set_ptr(void* f) {
    this->f = f;
  }

private:
  psl::string name_;
  TypeTag rtype_;
  psl::vector<TypeTag> ptypes_;
  void* f;
};

struct VariableConcept {
  virtual ~VariableConcept() = default;
  virtual psl::unique_ptr<VariableConcept> clone() = 0;
  virtual void* ptr() = 0;
  virtual size_t type_id() const = 0;
  virtual const psl::string& type_name() const = 0;
};

struct Variable {
  template <typename T>
  struct VariableModel : VariableConcept {
    VariableModel(T base) : base(MOVE(base)) {
    }
    psl::unique_ptr<VariableConcept> clone() override {
      return psl::make_unique<VariableModel>(base);
    }
    void* ptr() override {
      return &base;
    }
    size_t type_id() const override {
      return psl::type_id<T>();
    }
    const psl::string& type_name() const override {
      return psl::type_name<T>();
    }

  private:
    T base;
  };

  friend struct VariableConcept;

  Variable() : Variable(psl::Any()) {
  }
  template <typename T>
  Variable(T x) : model(psl::make_unique<VariableModel<T>>(MOVE(x))) {
  }
  Variable(psl::unique_ptr<VariableConcept> model) : model(MOVE(model)) {
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

  size_t type_id() const {
    return model->type_id();
  }
  const psl::string& type_name() const {
    return model->type_name();
  }
  void* ptr() const {
    return model->ptr();
  }

  template <typename T>
  bool is() const {
    return type_id() == psl::type_id<T>();
  }
  template <typename T>
  T as() const {
    using Base = psl::Decay<T>;
    return *reinterpret_cast<Base*>(ptr());
  }

private:
  psl::unique_ptr<VariableConcept> model;
};

struct TypeTrait {
  TypeTrait(psl::string name, size_t byte_size = 0) : name(MOVE(name)), byte_size(byte_size) {
  }
  bool is_fundamental() const {
    return members.size() == 0;
  }

  psl::string name;
  size_t byte_size = 0;
  psl::vector<TypeTrait*> members;
  void* destructor = nullptr;
};

template <typename T, typename U, typename R, typename... Args>
constexpr auto derived(R (U::*f)(Args...)) {
  return static_cast<R (T::*)(Args...)>(f);
}
template <typename T, typename U, typename R, typename... Args>
constexpr auto derived(R (U::*f)(Args...) const) {
  return static_cast<R (T::*)(Args...) const>(f);
}
template <typename... Args, typename R, typename T>
constexpr auto overloaded(R (T::*f)(Args...)) {
  return f;
}
template <typename... Args, typename R, typename T>
constexpr auto overloaded_const(R (T::*f)(Args...) const) {
  return f;
}
template <typename... Args, typename R>
constexpr auto overloaded(R (*f)(Args...)) {
  return f;
}
template <typename R, typename... Args>
constexpr auto overloaded_r(R (*f)(Args...)) {
  return f;
}

template <auto f, typename... Args>
auto tag_function() {
  return [](Args... args) { return f(FWD(args)...); };
}
template <auto f, typename... Ts>
struct Overloads {};
template <auto f, typename... Ts>
requires(sizeof...(Ts) > 0)
auto overloads() {
  return Overloads<f, Ts...>();
}

template <typename... Ts>
struct TypeSet {};
template <auto f, typename... Ts>
struct OverloadsSet {};
template <auto f, typename... Ts>
auto overloads_set() {
  return OverloadsSet<f, Ts...>();
}

template <auto f>
struct _lambdify_helper {
  auto operator+() const {
    return f;
  }
};
template <auto f>
constexpr _lambdify_helper<f> led;

struct Context {
  static Context context;

  template <typename T>
  struct TypeProxy {
    TypeProxy(Context& ctx, TypeTrait& trait) : ctx(ctx), trait(trait) {
    }

    template <typename... Ts>
    TypeProxy& layout() {
      trait.members = psl::vector_of(ctx.get_type_trait<Ts>()...);
      return *this;
    }
    template <auto ptr>
    TypeProxy& member(psl::string name) {
      ctx.add_f("@ma." + trait.name + "." + name, [](T& x) -> auto& { return x.*ptr; });
      return *this;
    }
    template <auto ptr>
    TypeProxy& method(psl::string name) {
      psl::overloaded_lambda(
          [&, this]<typename U, typename R, typename... Args>(R (U::*)(Args...)) {
            ctx.add_f(MOVE(name),
                      [](T& self, Args... args) -> R { return (self.*ptr)(FWD(args)...); });
          },
          [&, this]<typename U, typename R, typename... Args>(R (U::*)(Args...) const) {
            ctx.add_f(MOVE(name),
                      [](const T& self, Args... args) -> R { return (self.*ptr)(FWD(args)...); });
          })(ptr);
      return *this;
    }
    template <typename R, typename... Args>
    TypeProxy& method(psl::string name, R (*f)(Args...)) {
      ctx.add_f(MOVE(name), f);
      return *this;
    }
    template <typename... Args>
    TypeProxy& ctor(const char* name = nullptr) {
      ctx.add_f(name ? name : trait.name, [](Args... args) -> T { return T(FWD(args)...); });
      return *this;
    }
    template <typename Arg>
    TypeProxy& ctor(auto f) {
      ctx.add_f("@convert." + ctx.tag<Arg>().name + "." + trait.name, f);
      return *this;
    }

    template <auto converter, typename... Args>
    TypeProxy& ctor_variant_explicit() {
      (ctx.add_f(trait.name, [](Args arg) -> T { return converter(FWD(arg)); }), ...);
      return *this;
    }
    template <auto converter, typename... Args>
    TypeProxy& ctor_variant() {
      [&]<typename... Ts>(psl::IndexedTypeSequence<Ts...>) {
        ((ctx.add_f("@convert." + ctx.tag<typename Ts::Type>().name + "." + trait.name,
                    [](typename Ts::Type arg) -> T { return converter(FWD(arg)); }),
          ctx.add_f(trait.name, [](typename Ts::Type arg) -> T { return converter(FWD(arg)); }),
          ctx.add_f("=",
                    [](T& x, typename Ts::Type arg) -> T& { return x = converter(FWD(arg)); })),
         ...);
      }(psl::make_indexed_type_sequence<Args...>());

      return *this;
    }

    template <typename... Args>
    TypeProxy& ctor_variant_explicit() {
      constexpr auto default_ctor = [](auto&& x) -> T { return T(FWD(x)); };
      return ctor_variant_explicit<default_ctor, Args...>();
    }
    template <typename... Args>
    TypeProxy& ctor_variant() {
      constexpr auto default_ctor = [](auto&& x) -> T { return T(FWD(x)); };
      return ctor_variant<default_ctor, Args...>();
    }

  private:
    Context& ctx;
    TypeTrait& trait;
  };

  Context();

  struct FunctionProxy {
    void operator=(auto f) && {
      ctx.add_f(MOVE(name), MOVE(f));
    }
    template <auto f, typename... Ts>
    void operator=(Overloads<f, Ts...>) && {
      (ctx.add_f(name, tag_function<f, Ts>()), ...);
    }
    template <auto f, typename... Ts, typename... Us>
    void operator=(OverloadsSet<f, TypeSet<Ts...>, TypeSet<Us...>>) && {
      (
          [&]<typename U>() {
            (ctx.add_f(name, tag_function<f, Ts, U>()), ...);
          }.template operator()<Us>(),
          ...);
    }

    Context& ctx;
    psl::string name;
  };
  auto operator()(psl::string name) {
    return FunctionProxy{*this, MOVE(name)};
  }
  struct VariableProxy {
    void operator=(auto var) && {
      ctx.add_var(MOVE(name), MOVE(var));
    }
    Context& ctx;
    psl::string name;
  };
  auto var(psl::string name) {
    return VariableProxy{*this, MOVE(name)};
  }

  // ================================================
  // Functions
  // ================================================
  void add_var(psl::string name, auto var) {
    constants[name] = MOVE(var);
  }
  template <typename F>
  void add_f(psl::string name, F f) {
    static_assert(!psl::is_function<F>);
    add_f(name, wrap<F>(name, +f));
  }
  void add_f(psl::string name, Function func);
  void add_f(Function func) {
    functions.push_back(MOVE(func));
  }

  struct FindUniqueFResult {
    explicit operator bool() const {
      return fi != size_t(-1);
    }
    size_t fi;
    enum Error { Success, FindNone, FindTooMany } error;
  };
  FindUniqueFResult find_unique_f(psl::string_view name) const;
  struct FindFResult {
    explicit operator bool() const {
      return fi != size_t(-1);
    }
    size_t fi;
    struct ArgumentConversion {
      size_t position;
      size_t converter_index;
      TypeTag rtype;
    };
    psl::vector<ArgumentConversion> converts;
  };
  FindFResult find_f(psl::string_view name, psl::span<const TypeTag> atypes) const;

  template <typename R, typename... Args>
  R call(psl::string_view name, Args&&... args) const {
    if (auto fr = find_f(name, psl::array_of(tag<Args>()...))) {
      if (fr.converts.size())
        SEVERE("Arguments' type must exactly match that of the function");
      auto ptr = functions[fr.fi].ptr();
      if (psl::is_reference<R> || psl::is_void<R>) {
        using F = R (*)(Args&&...);
        return (F(ptr))(FWD(args)...);
      } else {
        using F = void (*)(R&, Args&&...);
        psl::Storage<sizeof(R), alignof(R)> res;
        (F(ptr))(*res.template ptr<R>(), FWD(args)...);
        return *res.template ptr<R>();
      }
    } else {
      SEVERE("Unable to call function `", name, "`");
    }
  }

  // ================================================
  // Types
  // ================================================
  template <typename T>
  auto type() {
    return TypeProxy<T>(*this, *get_type_trait<T>());
  }

  enum TypeClass { None, Float, Complex };
  template <typename T, TypeClass type_class = TypeClass::None>
  auto type(psl::string alias);
  TypeTrait* create_type_trait(psl::string name, size_t byte_size);
  TypeTrait* get_type_trait(psl::string_view name);
  const TypeTrait* get_type_trait(psl::string_view name) const;
  template <typename T>
  TypeTrait* get_type_trait() {
    return get_type_trait(name_of<T>());
  }
  template <typename T>
  const TypeTrait* get_type_trait() const {
    return get_type_trait(name_of<T>());
  }
  psl::string type_name_from_id(size_t type_id) const;
  template <typename T>
  psl::string name_of() const {
    if constexpr (psl::is_psl_function<T>)
      return name_of_function_type(reinterpret_cast<T*>(0));
    if (auto x = psl::find_or_nullopt(type_id_to_name, psl::type_id<T>()))
      return *x;
    else
      SEVERE("Type `", psl::type_name<T>(), "` is not registered");
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
    return TypeTag(name_of<T>(), psl::is_reference<T>,
                   psl::is_const_ref<T> || psl::is_const<T> || psl::is_rv_reference<T>);
  }
  template <typename... Ts>
  psl::vector<TypeTag> tags() const {
    return psl::vector_of<TypeTag>(tag<Ts>()...);
  }

  template <typename T>
  struct ParameterTypeTransform {
    using PublicType = T&&;
    using Type = T&&;

    static T get(T x) {
      return x;
    }
  };
  template <typename R, typename... Args>
  struct ParameterTypeTransform<psl::function<R(Args...)>> {
    using PublicType = psl::function<R(Args...)>;
    using Type = void*;

    static psl::function<R(Args...)> get(Type x) {
      if constexpr (psl::is_void<R>) {
        struct Lambda {
          Lambda(const Lambda&) = delete;
          Lambda& operator=(const Lambda&) = delete;
          Lambda(Lambda&& rhs) : obj(psl::exchange(rhs.obj, nullptr)), f(rhs.f) {
          }
          Lambda& operator=(Lambda&& rhs) {
            obj = psl::exchange(rhs.obj, nullptr);
            f = rhs.f;
            return *this;
          }
          ~Lambda() {
            if (obj)
              psl::free(obj);
          }

          using LambdaType = void (*)(void*, Args&&...);
          using FreeType = void (*)(Args&&...);
          void operator()(Args... args) const {
            if (obj)
              ((LambdaType)f)(obj, FWD(args)...);
            else
              ((FreeType)f)(FWD(args)...);
          }
          void* obj;
          void* f;
        };
        return MOVE(*(Lambda*)x);
      } else {
        struct Lambda {
          Lambda(const Lambda&) = delete;
          Lambda& operator=(const Lambda&) = delete;
          Lambda(Lambda&& rhs) : obj(psl::exchange(rhs.obj, nullptr)), f(rhs.f) {
          }
          Lambda& operator=(Lambda&& rhs) {
            obj = psl::exchange(rhs.obj, nullptr);
            f = rhs.f;
            return *this;
          }
          ~Lambda() {
            if (obj)
              psl::free(obj);
          }

          using LambdaType = void (*)(R*, void*, Args&&...);
          using FreeType = void (*)(R*, Args&&...);
          R operator()(Args... args) const {
            psl::Storage<sizeof(R), alignof(R)> res;
            if (obj)
              ((LambdaType)f)(res.template ptr<R>(), obj, FWD(args)...);
            else
              ((FreeType)f)(res.template ptr<R>(), FWD(args)...);
            return static_cast<R&&>(*res.template ptr<R>());
          }
          void* obj;
          void* f;
        };
        return MOVE(*(Lambda*)x);
      }
    }
  };
  template <typename F, typename R, typename... Args>
  Function wrap(psl::string name, R (*)(Args...)) const {
    if constexpr (psl::is_reference<R> || psl::is_void<R>) {
      constexpr auto transformed_f = +[](typename ParameterTypeTransform<Args>::Type... args) -> R {
        return (+F())(ParameterTypeTransform<Args>::get(args)...);
      };
      return Function(
          MOVE(name), tag<R>(),
          psl::vector_of<TypeTag>(tag<typename ParameterTypeTransform<Args>::PublicType>()...),
          (void*)transformed_f);
    } else {
      constexpr auto transformed_f =
          +[](R& res, typename ParameterTypeTransform<Args>::Type... args) -> void {
        psl::construct_at(&res, (+F())(ParameterTypeTransform<Args>::get(args)...));
      };
      return Function(
          MOVE(name), tag<R>(),
          psl::vector_of<TypeTag>(tag<typename ParameterTypeTransform<Args>::PublicType>()...),
          (void*)transformed_f);
    }
  }

  psl::string complete(psl::string part) const;

  psl::vector<Function> functions;
  psl::map<psl::string, psl::unique_ptr<TypeTrait>> types;
  psl::map<psl::string, Variable> constants;

private:
  psl::multimap<psl::string, size_t> function_name_to_index;
  psl::unordered_map<size_t, psl::string> type_id_to_name;
};

template <typename T, Context::TypeClass type_class>
auto Context::type(psl::string alias) {
  type_id_to_name[psl::type_id<T>()] = alias;
  auto byte_size = 0;
  if constexpr (!psl::is_void<T>)
    byte_size = sizeof(T);
  auto& trait = *types.insert({alias, psl::make_unique<TypeTrait>(alias, byte_size)}).first->second;

  if constexpr (!psl::is_void<T>) {
    if (!std::is_trivially_destructible_v<T>)
      trait.destructor = (void*)+[](T& x) { x.~T(); };
    if constexpr (psl::copyable<T> && !std::is_abstract_v<T>)
      add_f("@copy", [](T& x) { return x; });
    if constexpr (psl::movable<T> && !std::is_abstract_v<T>)
      add_f("@move", [](T& x) { return MOVE(x); });
  }

  if constexpr (type_class == TypeClass::Float || type_class == TypeClass::Complex) {
    add_f("+", [](T a, T b) { return a + b; });
    add_f("-", [](T a, T b) { return a - b; });
    add_f("*", [](T a, T b) { return a * b; });
    add_f("/", [](T a, T b) { return a / b; });
    add_f("+=", [](T& a, T b) -> T& { return a += b; });
    add_f("-=", [](T& a, T b) -> T& { return a -= b; });
    add_f("*=", [](T& a, T b) -> T& { return a *= b; });
    add_f("/=", [](T& a, T b) -> T& { return a /= b; });
    add_f("-x", [](T x) -> T { return -x; });
    add_f("==", [](T a, T b) -> bool { return a == b; });
    add_f("!=", [](T a, T b) -> bool { return a != b; });
    add_f("=", [](T& a, T b) -> T& { return a = b; });
    if constexpr (type_class == TypeClass::Float) {
      add_f("<", [](T a, T b) -> bool { return a < b; });
      add_f(">", [](T a, T b) -> bool { return a > b; });
      add_f("<=", [](T a, T b) -> bool { return a <= b; });
      add_f(">=", [](T a, T b) -> bool { return a >= b; });
    }
  }
  return TypeProxy<T>(*this, trait);
}

}  // namespace pine