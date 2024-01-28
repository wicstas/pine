#pragma once

#include <pine/core/log.h>

#include <psl/type_traits.h>
#include <psl/optional.h>
#include <psl/string.h>
#include <psl/vector.h>
#include <psl/system.h>
#include <psl/map.h>

namespace pine {

enum class Qualifier { One, Maybe, OneOrMore, Many };
inline psl::string to_string(Qualifier qualifier) {
  switch (qualifier) {
    case Qualifier::One:
      return "";
    case Qualifier::Maybe:
      return "?";
    case Qualifier::OneOrMore:
      return "+";
    case Qualifier::Many:
      return "*";
  }
}

struct GrammarItem {
  psl::string sym;
  Qualifier qualifier;
  bool superficial = false;
};

struct GrammarRule {
  psl::string sym;
  psl::vector<GrammarItem> expr;
};

template <typename T>
constexpr bool is_grammar_item = false;
template <typename T>
concept GrammarItem_ = is_grammar_item<T>;

struct Sym {
  Sym(psl::string sym) : sym(psl::move(sym)) {
  }
  psl::string sym;
};

inline Sym operator""_s(const char *str, size_t len) {
  return Sym(psl::string(str, len));
}

struct Maybe {
  Maybe(Sym item) : sym(item.sym){};
  psl::string sym;
};
struct OneOrMore {
  OneOrMore(Sym item) : sym(item.sym){};
  psl::string sym;
};
struct Many {
  Many(Sym item) : sym(item.sym){};
  psl::string sym;
};
template <GrammarItem_ T>
struct Sup {
  Sup(T item) : item(item) {
  }
  T item;
};

template <typename T, typename U>
struct Concat {
  Concat(T first, U second) : first(first), second(second) {
  }
  T first;
  U second;
};

template <>
inline constexpr bool is_grammar_item<Sym> = true;
template <>
inline constexpr bool is_grammar_item<Maybe> = true;
template <>
inline constexpr bool is_grammar_item<OneOrMore> = true;
template <>
inline constexpr bool is_grammar_item<Many> = true;
template <typename T, typename U>
inline constexpr bool is_grammar_item<Concat<T, U>> = true;
template <typename T>
inline constexpr bool is_grammar_item<Sup<T>> = true;

template <GrammarItem_ T, GrammarItem_ U>
auto operator+(T first, U second) {
  return Concat<T, U>(first, second);
}

class Grammar {
public:
  struct RuleProxy {
    RuleProxy(Grammar &grammar, size_t rule_index) : grammar(grammar), rule_index(rule_index) {
    }
    void operator=(GrammarItem_ auto item) {
      parse(item);
    };

  private:
    void parse(Sym sym) {
      add_item(GrammarItem{sym.sym, Qualifier::One, superficial});
    };
    void parse(Maybe item) {
      auto qualified_sym = "@maybe#" + psl::to_string(item.sym);
      add_item(GrammarItem{qualified_sym, Qualifier::One, superficial});
      grammar.add_qualified(item.sym, qualified_sym, Qualifier::Maybe);
    };
    void parse(OneOrMore item) {
      auto qualified_sym = "@oneOrMore#" + psl::to_string(item.sym);
      add_item(GrammarItem{qualified_sym, Qualifier::One, superficial});
      grammar.add_qualified(item.sym, qualified_sym, Qualifier::OneOrMore);
    }
    void parse(Many item) {
      auto qualified_sym = "@many#" + psl::to_string(item.sym);
      add_item(GrammarItem{qualified_sym, Qualifier::One, superficial});
      grammar.add_qualified(item.sym, qualified_sym, Qualifier::Many);
    }
    template <typename T>
    void parse(Sup<T> x) {
      superficial = true;
      parse(x.item);
      superficial = false;
    }
    template <typename T, typename U>
    void parse(Concat<T, U> x) {
      parse(x.first);
      parse(x.second);
    }

    void add_item(GrammarItem item) {
      grammar.rules[rule_index].expr.push_back(item);
    }

    Grammar &grammar;
    size_t rule_index;
    bool superficial = false;
  };

  RuleProxy operator[](psl::string sym) {
    auto rule_index = rules.size();
    rules.push_back(GrammarRule{sym, {}});
    sym2index.insert({sym, rule_index});
    return RuleProxy{*this, rule_index};
  }

  void add_qualified(psl::string sym, psl::string qualified_sym, Qualifier qualifier) {
    if (sym2index.find(qualified_sym) == sym2index.end()) {
      auto rule_index = rules.size();
      rules.push_back(
          GrammarRule{qualified_sym, psl::vector_of(GrammarItem{sym, qualifier, false})});
      sym2index.insert({qualified_sym, rule_index});
    }
  }

  psl::vector<GrammarRule> rules;
  psl::multimap<psl::string, size_t> sym2index;
};

inline psl::string to_string(const Grammar &grammar) {
  auto str = psl::string();
  for (const auto &rule : grammar.rules) {
    str += rule.sym + " := ";
    for (const auto &item : rule.expr)
      str += item.sym + to_string(item.qualifier) + " ";
    str += "\n";
  }
  return str;
};

struct EarleyNode {
  psl::string sym;
  psl::vector<EarleyNode> children;
};

inline psl::string to_string(const EarleyNode &node, int depth = 0) {
  auto str = psl::string(depth * 2, ' ');
  str += node.sym + "\n";

  for (const auto &child : node.children)
    str += to_string(child, depth + 1);

  return str;
}

psl::optional<EarleyNode> Parse(const Grammar &grammar, const psl::vector<EarleyNode> &tokens,
                                const psl::string &root_sym);

template <typename... Ts>
struct CtorSpecifier {};
template <typename... Ts>
constexpr auto Ctor = CtorSpecifier<Ts...>{};

struct UseString {};

template <typename... Ts>
struct CheckUseString : psl::FalseType {};
template <>
struct CheckUseString<UseString> : psl::TrueType {};

struct Object {
  struct Concept {
    virtual ~Concept() = default;
    virtual psl::unique_ptr<Concept> make_optional() const = 0;
    virtual psl::unique_ptr<Concept> make_empty_vector() const = 0;
    virtual void move_append(Object &vector) = 0;
    virtual psl::unique_ptr<Concept> clone() const = 0;
    virtual void *ptr() = 0;
    virtual size_t type_id() const = 0;
    virtual const psl::string &type_name() const = 0;
  };
  template <typename T, bool final_ = false>
  struct Model : Concept {
    Model() = default;
    Model(T base) : base{psl::move(base)} {
    }
    psl::unique_ptr<Concept> make_optional() const override {
      CHECK(!final_);
      if constexpr (!final_)
        return psl::make_unique<Model<psl::optional<T>, true>>(base);
      else
        return nullptr;
    }
    psl::unique_ptr<Concept> make_empty_vector() const override {
      if constexpr (!final_)
        return psl::make_unique<Model<psl::vector<T>, true>>();
      else
        return nullptr;
    }
    void move_append(Object &vector) override {
      CHECK(!final_);
      vector.as<psl::vector<T>>().push_back(psl::move(base));
    }
    psl::unique_ptr<Concept> clone() const override {
      return psl::make_unique<Model>(*this);
    }
    void *ptr() override {
      return reinterpret_cast<void *>(&base);
    }
    size_t type_id() const override {
      return psl::type_id<T>();
    }
    const psl::string &type_name() const override {
      return psl::type_name<T>();
    }

  private:
    T base;
  };

  Object() = default;
  template <typename T>
  Object(T base) : model(psl::make_unique<Model<T>>(psl::move(base))) {
  }

  Object(Object &&) = default;
  Object(const Object &rhs) {
    if (rhs.model)
      model = rhs.model->clone();
  }
  Object &operator=(Object rhs) {
    model = psl::move(rhs.model);
    return *this;
  }

  Object make_optional() const {
    CHECK(model != nullptr);
    auto x = Object{};
    x.model = model->make_optional();
    return x;
  }
  Object make_empty_vector() const {
    CHECK(model != nullptr);
    auto x = Object{};
    x.model = model->make_empty_vector();
    return x;
  }
  void move_append(Object &vector) {
    CHECK(model != nullptr);
    model->move_append(vector);
  }

  const psl::string &type_name() const {
    CHECK(model != nullptr);
    return model->type_name();
  }

  template <typename T>
  bool is() const {
    CHECK(model != nullptr);
    return model->type_id() == psl::type_id<T>();
  }
  template <typename T>
  T &as() {
    CHECK(model != nullptr);
    if (!is<T>())
      Fatal("Can't interpret ", type_name(), " as ", psl::type_name<T>());
    return *reinterpret_cast<T *>(model->ptr());
  }
  template <typename T>
  const T &as() const {
    CHECK(model != nullptr);
    if (!is<T>())
      Fatal("Can't interpret ", type_name(), " as ", psl::type_name<T>());
    return *reinterpret_cast<const T *>(model->ptr());
  }

  psl::unique_ptr<Concept> model;
};

struct Generator {
  struct Concept {
    Concept(bool use_string) : use_string{use_string} {
    }

    virtual ~Concept() = default;
    virtual Object make_null_optional() const = 0;
    virtual Object make_empty_vector() const = 0;
    virtual psl::optional<Object> construct(const psl::vector<Object> &args) const = 0;
    virtual psl::optional<Object> construct(psl::string arg) const = 0;
    virtual const psl::string &type_name() const = 0;

    bool use_string = false;
  };

  template <typename T, typename... Ctors>
  struct Model : Concept {
    using Concept::Concept;

    psl::optional<Object> construct(const psl::vector<Object> &args) const override {
      if constexpr (CheckUseString<Ctors...>::value) {
        return psl::nullopt;
      } else {
        if constexpr (sizeof...(Ctors) == 0)
          return Object{T{}};
        else
          return match(args, Ctors{}...);
      }
    }
    psl::optional<Object> construct(psl::string expr) const override {
      if constexpr (CheckUseString<Ctors...>::value)
        return Object{T(expr)};
      else
        return psl::nullopt;
    }

    Object make_null_optional() const override {
      return Object{psl::optional<T>{psl::nullopt}};
    }
    Object make_empty_vector() const override {
      return Object{psl::vector<T>{}};
    }

    const psl::string &type_name() const override {
      return psl::type_name<T>();
    }

    template <typename... Args, int... I>
    static bool is(const psl::vector<Object> &args, psl::IntegerSequence<int, I...>) {
      return (args[I].is<psl::NthType<I, Args...>>() && ...);
    }

    template <typename... Args, int... I>
    static Object constructBy(const psl::vector<Object> &args, psl::IntegerSequence<int, I...>) {
      return Object{T(args[I].as<psl::NthType<I, Args...>>()...)};
    }

    template <typename... Args, typename... Cs>
    static psl::optional<Object> match(const psl::vector<Object> &args, CtorSpecifier<Args...>,
                                       Cs... ctors) {
      const auto seq = psl::make_integer_sequence<int, sizeof...(Args)>();
      if (args.size() == sizeof...(Args) && is<Args...>(args, seq))
        return constructBy<Args...>(args, seq);

      if constexpr (sizeof...(Cs) == 0)
        return psl::nullopt;
      else
        return match(args, ctors...);
    }
  };

  template <typename T, typename... Ctors>
  void bind(psl::string name, Ctors...) {
    models[name] = psl::make_unique<Model<T, Ctors...>>(false);
  }
  template <typename T>
  void bind(psl::string name, UseString) {
    models[name] = psl::make_unique<Model<T, UseString>>(true);
  }

  Concept *find_model(psl::string sym) const {
    auto it = models.find(sym);
    if (it == psl::end(models))
      Fatal("Cannot find the model for `", sym, '`');
    return it->second.get();
  }

private:
  std::map<psl::string, psl::unique_ptr<Concept>> models;
};

Object generate_impl(const Generator &generator, const EarleyNode &node);

template <typename T>
T generate(const Generator &generator, const EarleyNode &node) {
  return generate_impl(generator, node).as<T>();
}

}  // namespace pine