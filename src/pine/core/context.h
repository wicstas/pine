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

template <typename T, size_t max_static_bytes>
struct smart_unique_ptr {
  template <typename U>
  smart_unique_ptr(psl::unique_ptr<U> rhs) : ptr(psl::unique_ptr<T>(MOVE(rhs))), is_local(false) {
  }
  template <typename U>
  smart_unique_ptr(U x) : is_local(true) {
    static_assert(sizeof(U) <= max_static_bytes, "");
    psl::construct_at(local.template ptr<U>(), MOVE(x));
  }
  ~smart_unique_ptr() {
    if (is_local)
      psl::destruct_at(local.template ptr<T>());
  }
  smart_unique_ptr(smart_unique_ptr&& rhs) {
    if ((is_local = psl::exchange(rhs.is_local, false)))
      local = rhs.local;
    else
      ptr = MOVE(rhs.ptr);
  }
  smart_unique_ptr& operator=(smart_unique_ptr&& rhs) {
    if ((is_local = psl::exchange(rhs.is_local, false)))
      local = rhs.local;
    else
      ptr = MOVE(rhs.ptr);
    return *this;
  }

  T* get() {
    return is_local ? local.template ptr<T>() : ptr.get();
  }
  T* get() const {
    return is_local ? local.template ptr<T>() : ptr.get();
  }
  T& operator*() {
    return *get();
  }
  const T& operator*() const {
    return *get();
  }
  T* operator->() {
    return get();
  }
  T* operator->() const {
    return get();
  }

private:
  psl::unique_ptr<T> ptr;
  psl::Storage<max_static_bytes, 8> local;
  bool is_local;
};

struct TypeTag {
  TypeTag() = default;
  TypeTag(psl::string name, bool is_ref = false, bool is_const = false)
      : name(MOVE(name)), is_ref(is_ref), is_const(is_const) {
  }

  psl::string sig() const {
    return (is_const ? "const " : "") + name + (is_ref ? "&" : "");
  }

  friend bool operator==(const TypeTag& lhs, const TypeTag& rhs) {
    return lhs.name == rhs.name && lhs.is_ref == rhs.is_ref && lhs.is_const == rhs.is_const;
  }

  psl::string name;
  bool is_ref = false;
  bool is_const = false;
};
inline psl::string to_string(const TypeTag& type) {
  return type.sig();
}

template <typename F, typename R, typename... Args>
struct Lambda {
  Lambda(F lambda) : lambda(MOVE(lambda)) {
  }
  F lambda;
};
template <typename R, typename... Args, typename F>
auto tag(F lambda) {
  return Lambda<F, R, Args...>(MOVE(lambda));
}
template <typename... Args, typename F>
auto lambda(F lambda) {
  return Lambda<F, psl::ReturnType<F, Args...>, Args...>(MOVE(lambda));
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

template <typename T>
auto foward_ref(psl::TypeIdentity<T>& x) {
  if constexpr (psl::is_reference<T>)
    return psl::ref(x);
  else
    return x;
}

struct VariableConcept {
  static constexpr size_t max_static_bytes = 16;
  using ModelPtr = smart_unique_ptr<VariableConcept, max_static_bytes>;

  virtual ~VariableConcept() = default;
  virtual ModelPtr clone() = 0;
  virtual ModelPtr copy() = 0;
  virtual ModelPtr create_ref() = 0;
  virtual void* ptr() = 0;
  virtual size_t type_id() const = 0;
  virtual const psl::string& type_name() const = 0;
};

struct Variable {
  template <typename R, typename T>
  struct VariableModel : VariableConcept {
    VariableModel(T base) : base{MOVE(base)} {
    }
    ModelPtr clone() override {
      return psl::make_unique<VariableModel>(base);
    }
    ModelPtr copy() override {
      if constexpr (psl::copyable<R>) {
        if constexpr (psl::is_psl_ref<T>)
          return psl::make_unique<VariableModel<R, R>>(*base);
        else
          return psl::make_unique<VariableModel<R, R>>(base);
      } else {
        PINE_UNREACHABLE;
      }
    }
    ModelPtr create_ref() override {
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
  Variable(T x)
      : model([&]() -> VariableConcept::ModelPtr {
          if constexpr (sizeof(VariableModel<T, T>) > VariableConcept::max_static_bytes)
            return psl::make_unique<VariableModel<T, T>>(MOVE(x));
          else
            return VariableModel<T, T>(MOVE(x));
        }()) {
  }
  template <typename T>
  Variable(psl::Ref<T> x) : model(VariableModel<T, psl::Ref<T>>(MOVE(x))) {
  }
  Variable(VariableConcept::ModelPtr model) : model(MOVE(model)) {
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
#ifndef NDEBUG
    if (!is<Base>())
      Fatal("Trying to interpret ", type_name(), " as ", psl::type_name<T>());
#endif
    return *reinterpret_cast<Base*>(model->ptr());
  }

  void* ptr() const {
    return model->ptr();
  }

private:
  VariableConcept::ModelPtr model;
};

psl::string signature_from(const TypeTag& rtype, psl::span<const TypeTag> ptypes);

struct Function {
  struct FunctionConcept {
    virtual ~FunctionConcept() = default;

    virtual Variable call(psl::span<const Variable*> args) = 0;
    virtual TypeTag rtype() const = 0;
    virtual psl::span<const TypeTag> ptypes() const = 0;
    virtual const psl::string& name() const = 0;

    virtual void* ptr() const = 0;
    virtual void* object_ptr() const = 0;
    virtual size_t byte_size() const = 0;
  };

  template <typename Repr, typename F>
  struct FunctionModel;
  template <typename Repr, typename R, typename... Args>
  struct FunctionModel<Repr, R(Args...)> : FunctionConcept {
    static_assert(sizeof...(Args) <= 8, "Function can only have up to 8 parameters");

    FunctionModel(Repr f, TypeTag rtype, psl::vector<TypeTag> ptypes, psl::string name)
        : f(MOVE(f)), rtype_(MOVE(rtype)), ptypes_(MOVE(ptypes)), name_(MOVE(name)) {
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
    const psl::string& name() const override {
      return name_;
    }
    void* ptr() const override {
      return (void*)+[](uint8_t* ptr) {
        auto& f = *(Repr*)ptr;
        ptr += sizeof(Repr);
        using R_ = psl::RemoveReference<R>;
        if constexpr (psl::is_void<R_>)
          psl::apply(*(psl::tuple<Args...>*)(ptr), f);
        else
          new ((R_*)ptr) R_(psl::apply(*(psl::tuple<Args...>*)(ptr + sizeof(R_)), f));
      };
    }
    void* object_ptr() const override {
      return (void*)&f;
    }
    size_t byte_size() const override {
      return sizeof(Repr);
    }

  private:
    Repr f;
    TypeTag rtype_;
    psl::vector<TypeTag> ptypes_;
    psl::string name_;
  };

  template <typename R, typename... Args>
  Function(R (*f)(Args...), TypeTag rtype, psl::vector<TypeTag> ptypes, psl::string name = "<>")
      : model(psl::make_shared<FunctionModel<R (*)(Args...), R(Args...)>>(
            f, MOVE(rtype), MOVE(ptypes), MOVE(name))) {
  }
  template <typename T, typename R, typename... Args>
  Function(Lambda<T, R, Args...> f, TypeTag rtype, psl::vector<TypeTag> ptypes,
           psl::string name = "<>")
      : model(psl::make_shared<FunctionModel<T, R(Args...)>>(MOVE(f.lambda), MOVE(rtype),
                                                             MOVE(ptypes), MOVE(name))) {
  }

  Variable call(psl::span<const Variable*> args) const {
    return model->call(args);
  }
  Variable operator()(auto&&... args) const {
    Variable args_[]{Variable(foward_ref<decltype(args)>(args))...};
    const Variable* args_ptr[sizeof...(args)];
    for (size_t i = 0; i < sizeof...(args); i++)
      args_ptr[i] = &args_[i];
    return call(args_ptr);
  }
  const psl::string& name() const {
    return model->name();
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
    return name() + signature_from(rtype(), ptypes());
  }
  void* ptr() const {
    return model->ptr();
  }
  void* object_ptr() const {
    return model->object_ptr();
  }
  size_t byte_size() const {
    return model->byte_size();
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
  return Overloads<F, Ts...>(MOVE(f));
}

template <typename F, typename... Ts>
struct OverloadsSet {
  F f;
};
template <typename... Ts, typename F>
auto overloads_set(F f) {
  return OverloadsSet<F, Ts...>(MOVE(f));
}

struct Context {
  struct Proxy {
    Proxy(Context& ctx, psl::string name) : ctx(ctx), name(MOVE(name)) {
    }
    void operator=(auto x) const {
      add(MOVE(x));
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
      ctx.add_f(MOVE(name), f);
    }
    template <typename F, typename R, typename... Args>
    void add(Lambda<F, R, Args...> lambda) const {
      ctx.add_f(MOVE(name), MOVE(lambda));
    }
    void add(Function f) const {
      ctx.add_f(MOVE(name), MOVE(f));
    }
    template <typename T>
    requires requires { &T::operator(); }
    void add(T f) const {
      ctx.add_f(MOVE(name), MOVE(f));
    }
    template <typename T>
    void add(T value) const {
      ctx.add_variable(MOVE(name), MOVE(value));
    }

  private:
    Context& ctx;
    psl::string name;
  };

  struct TypeTrait {
    TypeTrait() = default;
    TypeTrait(psl::string type_name, size_t byte_size)
        : type_name_(MOVE(type_name)), byte_size_(byte_size) {
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
    size_t byte_size() const {
      return byte_size_;
    }

    void* copy_operator = nullptr;

  private:
    psl::string type_name_;
    psl::map<psl::string, size_t> member_accessors;
    psl::map<psl::string, size_t> from_converters;
    size_t byte_size_;
  };

  enum TypeClass { None, Float, Complex };

  template <typename T>
  struct TypeProxy {
    TypeProxy(Context& ctx, TypeTrait& trait) : ctx(ctx), trait(trait) {
    }
    template <typename U>
    TypeProxy& member(psl::string name, U T::*ptr) {
      trait.add_member_accessor(name, ctx.functions.size());
      ctx.add_f(".", pine::tag<U&, T&>([ptr](T& x) -> U& { return x.*ptr; }));
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
      ctx.add_f(trait.type_name(), +[](Args... args) { return T(FWD(args)...); });
      return *this;
    }
    template <typename... Args>
    TypeProxy& ctor(psl::string name) {
      ctx.add_f(name, +[](Args... args) { return T(FWD(args)...); });
      return *this;
    }
    template <typename F>
    TypeProxy& ctor(F f) {
      ctx.add_f(trait.type_name(), MOVE(f));
      return *this;
    }
    template <typename... Args>
    TypeProxy& ctor_variant_explicit() {
      (ctx.add_f(trait.type_name(), +[](Args arg) { return T(static_cast<Args&&>(arg)); }), ...);
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
      (ctx.add_f(trait.type_name(), +[](Args arg) { return T(static_cast<Args&&>(arg)); }), ...);
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
    return Proxy(*this, MOVE(name));
  }
  template <typename T>
  auto type() {
    return TypeProxy<T>(*this, get_type_trait<T>());
  }
  template <typename T, TypeClass type_class = TypeClass::None>
  auto type(psl::string alias);

  template <typename F>
  void add_f(psl::string name, F f) {
    auto func = wrap(name, f);
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
    functions_map.insert({MOVE(name), functions.size()});
    add_f(MOVE(func));
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
    variables_map.insert({MOVE(name), variables.size()});
    variables.push_back({tag<T>(), MOVE(x)});
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
    return TypeTag(name_of<T>(), psl::is_reference<T>, psl::is_const_ref<T> || psl::is_const<T>);
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
  psl::vector<TypeTag> rtype_stack;

private:
  Function wrap(psl::string, Function f) const {
    return f;
  }
  template <typename R, typename... Args>
  Function wrap(psl::string name, R (*f)(Args...)) const {
    return Function(f, tag<R>(), psl::vector_of<TypeTag>(tag<Args>()...), MOVE(name));
  }
  template <typename T, typename R, typename... Args>
  Function wrap(psl::string name, R (T::*f)(Args...)) const {
    auto lambda = pine::tag<R, T&, Args...>(
        [f](T& x, Args... args) -> R { return (x.*f)(static_cast<Args&&>(args)...); });
    return Function(MOVE(lambda), tag<R>(), psl::vector_of(tag<T&>(), tag<Args>()...), MOVE(name));
  }
  template <typename T, typename R, typename... Args>
  Function wrap(psl::string name, R (T::*f)(Args...) const) const {
    auto lambda = pine::tag<R, const T&, Args...>(
        [f](const T& x, Args... args) -> R { return (x.*f)(static_cast<Args&&>(args)...); });
    return Function(MOVE(lambda), tag<R>(), psl::vector_of(tag<const T&>(), tag<Args>()...),
                    MOVE(name));
  }
  template <typename T, typename R, typename... Args>
  Function wrap(psl::string name, Lambda<T, R, Args...> f) const {
    return Function(MOVE(f), tag<R>(), psl::vector_of<TypeTag>(tag<Args>()...), MOVE(name));
  }
  template <typename T>
  Function wrap(psl::string name, T lambda) const {
    return wrap(MOVE(name), MOVE(lambda), &T::operator());
  }
  template <typename T, typename R, typename... Args>
  Function wrap(psl::string name, T f, R (T::*)(Args...)) const {
    return Function(Lambda<T, R, Args...>(MOVE(f)), tag<R>(),
                    psl::vector_of<TypeTag>(tag<Args>()...), MOVE(name));
  }
  template <typename T, typename R, typename... Args>
  Function wrap(psl::string name, T f, R (T::*)(Args...) const) const {
    return Function(Lambda<T, R, Args...>(MOVE(f)), tag<R>(),
                    psl::vector_of<TypeTag>(tag<Args>()...), MOVE(name));
  }
};

template <typename T, Context::TypeClass type_class>
auto Context::type(psl::string alias) {
  types_map[psl::type_id<T>()] = alias;
  auto& trait = [&]() -> TypeTrait& {
    if constexpr (psl::is_void<T>)
      return types[alias] = TypeTrait(alias, 0);
    else
      return types[alias] = TypeTrait(alias, sizeof(T));
  }();
  if constexpr (psl::copyable<T>)
    trait.copy_operator = (void*)+[](T x) { return x; };
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