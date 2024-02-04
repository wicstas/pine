#include <pine/core/context.h>

#include <psl/algorithm.h>

namespace pine {

struct Any {};

Context::Context() {
  auto& context = *this;
  context.type<Any>("any");
  context.type<Variable>("variable");
  context.type<psl::Empty>("void");
  context.type<void>("void");
  context.type<Function>("function");
  context.type<psl::span<const Variable*>>("__va_args");

  context.type<psl::string>("str");
  context("=") = +[](psl::string& a, psl::string b) -> psl::string& { return a = b; };
  context("+=") = +[](psl::string& a, psl::string b) -> psl::string& { return a += b; };
  context("+") = +[](psl::string a, psl::string b) { return a + b; };

  context.type<bool>("bool");
  add_f(
      "==", +[](bool a, bool b) -> bool { return a == b; });
  add_f(
      "!=", +[](bool a, bool b) -> bool { return a != b; });
  add_f(
      "=", +[](bool& a, bool b) -> bool& { return a = b; });

  decltype(auto) ftype = context.type<float, Context::Float>("f32");
  context.type<uint16_t>("u16");
  context.type<uint32_t>("u32");
  context.type<uint64_t>("u64");
  context.type<int64_t>("i64");
  context.type<int32_t, Context::Float>("i32").ctor_variant_explicit<float>();
  context("^") = +[](int a, int b) { return psl::powi(a, b); };
  context("++x") = +[](int& x) -> decltype(auto) { return ++x; };
  context("--x") = +[](int& x) -> decltype(auto) { return --x; };
  context("x++") = +[](int& x) -> decltype(auto) { return x++; };
  context("x--") = +[](int& x) -> decltype(auto) { return x--; };
  context("%") = +[](int a, int b) { return a % b; };
  context("%=") = +[](int& a, int b) -> int& { return a %= b; };
  context("+") = +[](int a, float b) { return a + b; };
  context("-") = +[](int a, float b) { return a - b; };
  context("*") = +[](int a, float b) { return a * b; };
  context("/") = +[](int a, float b) { return a / b; };
  context("+") = +[](float a, int b) { return a + b; };
  context("-") = +[](float a, int b) { return a - b; };
  context("*") = +[](float a, int b) { return a * b; };
  context("/") = +[](float a, int b) { return a / b; };
  context("+=") = +[](int& a, float b) -> int& { return a += b; };
  context("-=") = +[](int& a, float b) -> int& { return a -= b; };
  context("*=") = +[](int& a, float b) -> int& { return a *= b; };
  context("/=") = +[](int& a, float b) -> int& { return a /= b; };
  context("+=") = +[](float& a, int b) -> float& { return a += b; };
  context("-=") = +[](float& a, int b) -> float& { return a -= b; };
  context("*=") = +[](float& a, int b) -> float& { return a *= b; };
  context("/=") = +[](float& a, int b) -> float& { return a /= b; };

  ftype.ctor_variant<int32_t>();
  context("^") = +[](float a, float b) { return psl::pow(a, b); };

  context("=") = Function(
      +[](psl::span<const Variable*> args) { args[0]->as<Variable&>() = *args[1]; },
      context.tag<void>(), context.tags<Variable&, Any>());

  struct Type {
    psl::string name;
    psl::string raw_name;
    uint64_t id;
  };
  context.type<Type>("Type")
      .member("name", &Type::name)
      .member("raw_name", &Type::raw_name)
      .member("id", &Type::id);
  context("type") =
      Function(lambda<psl::span<const Variable*>>([&context](psl::span<const Variable*> args) {
                 return Type{context.name_from_id(args[0]->type_id()), args[0]->type_name(),
                             args[0]->type_id()};
               }),
               context.tag<Type>(), context.tags<Any>());

  context("apply_to_5") = +[](psl::function<int, int> f) { return f(5); };
}

static int common_prefix_length(psl::string_view a, psl::string_view b) {
  auto i = size_t(0);
  for (; i < psl::min(a.size(), b.size()); i++) {
    if (a[i] != b[i])
      break;
  }
  return i;
}

static int common_part_length(psl::string_view a, psl::string_view b) {
  auto max_len = 0;
  for (size_t i = 0; i < b.size(); i++) {
    max_len = psl::max(common_prefix_length(a, b.substr(i)), max_len);
  }
  return max_len;
}

static psl::string function_signature(const Function& function) {
  auto sig = psl::string("(");
  for (const auto& ptype : function.ptypes())
    sig += ptype.name + ", ";
  if (function.ptypes().size() != 0)
    sig.pop_back(2);
  sig += "): " + function.rtype().name;
  return sig;
}

Context::FindUniqueFResult Context::find_unique_f(psl::string_view name) const {
  auto result = FindUniqueFResult();
  auto [first, last] = functions_map.equal_range(name);
  if (first == last) {
    result.status = FindUniqueFResult::None;
  } else {
    --last;
    if (first == last) {
      result.status = FindUniqueFResult::Found;
      result.function_index = first->second;
    } else {
      result.status = FindUniqueFResult::TooMany;
      ++last;
      for (auto [name, fi] : psl::range(first, last))
        result.candidates += name + function_signature(functions[fi]) + "\n";
      result.candidates.pop_back();
    }
  }

  return result;
}

Context::FindFResult Context::find_f(psl::string_view name, psl::span<const TypeTag> atypes) const {
  auto [first, last] = functions_map.equal_range(name);
  auto best_coeff = atypes.size();
  auto best_candidates = psl::vector<size_t>();
  auto best_converts = psl::vector<Context::FindFResult::ArgumentConversion>();

  auto arg_signature = psl::string();
  for (auto atype : atypes)
    arg_signature += atype.name + ", ";
  if (arg_signature.size()) {
    arg_signature.pop_back();
    arg_signature.pop_back();
  }

  for (auto [name, fi] : psl::range(first, last)) {
    const auto& ptypes = functions[fi].ptypes();
    if (ptypes.size() == 1 && ptypes[0].name == "__va_args") {
      best_converts = {};
      best_candidates.clear();
      best_candidates.push_back(fi);
      break;
    } else if (atypes.size() == ptypes.size()) {
      auto converts = psl::vector<Context::FindFResult::ArgumentConversion>();
      auto difference = size_t{0};
      for (size_t i = 0; i < atypes.size(); i++) {
        if (atypes[i].name == ptypes[i].name) {
        } else if (ptypes[i].name == "any") {
        } else if (is_registered_type(ptypes[i].name)) {
          if (auto ci = get_type_trait(ptypes[i].name).find_from_converter_index(atypes[i].name);
              ci != size_t(-1)) {
            converts.push_back({i, ci, ptypes[i]});
            difference += 1;
          } else {
            difference = atypes.size() + 1;
            break;
          }
        } else {
          difference = atypes.size() + 1;
          break;
        }
      }
      if (difference <= best_coeff) {
        if (difference < best_coeff) {
          best_candidates.clear();
        }
        best_coeff = difference;
        best_candidates.push_back(fi);
        best_converts = converts;
      }
    }
  }

  if (best_candidates.size() == 1) {
    auto fi = best_candidates.front();
    // Debug("Best candicate of `", name, "` is ", function_signature(functions[fi]));
    return {fi, functions[fi].rtype(), best_converts};
  } else if (best_candidates.size() > 1) {
    auto candidates = psl::string();
    for (auto fi : best_candidates)
      candidates += name + function_signature(functions[fi]) + "\n";
    if (candidates.size())
      candidates.pop_back();
    exception("Ambiguous function call `", name, "(", arg_signature, ")`, candidates:\n",
              candidates);
  } else {
    if (first != last) {
      auto candidates = psl::string();
      for (auto [name, fi] : psl::range(first, last))
        candidates += name + function_signature(functions[fi]) + "\n";
      if (candidates.size())
        candidates.pop_back();
      exception("Function `", name, "(", arg_signature, ")` is not found, candidates:\n",
                candidates);
    } else {
      auto likely_func_name = psl::string();
      auto max_common_part_length = 0;
      for (const auto& f : functions_map) {
        if (auto len = common_part_length(name, f.first); len > max_common_part_length) {
          likely_func_name = f.first;
          max_common_part_length = len;
        }
      }

      if (likely_func_name != "")
        exception("Function `", name, "(", arg_signature, ")` is not found, did you mean `",
                  likely_func_name, "`?");
      else
        exception("Function `", name, "(", arg_signature, ")` is not found");
    }
  }
}

Variable Context::call(psl::string_view name, psl::span<const TypeTag> atypes,
                       psl::vector<Variable> args) const {
  auto fr = find_f(name, atypes);
  for (const auto& cv : fr.converts) {
    args[cv.position] = functions[cv.converter_index](args[cv.position]);
  };
  auto args_ = psl::vector<const Variable*>(args.size());
  for (size_t i = 0; i < args.size(); i++)
    args_[i] = &args[i];
  return functions[fr.function_index].call(args_);
}

void Context::add_f(Function func) {
  functions.push_back(psl::move(func));
}

size_t Context::find_variable(psl::string_view name) const {
  if (auto it = variables_map.find(name); it != variables_map.end())
    return it->second;
  else
    return size_t(-1);
}

Context::TypeTrait& Context::get_type_trait(psl::string_view name) {
  if (auto it = types.find(name); it != types.end())
    return it->second;
  else
    exception("Type `", name, "` not register");
}
const Context::TypeTrait& Context::get_type_trait(psl::string_view name) const {
  if (auto it = types.find(name); it != types.end())
    return it->second;
  else
    exception("Type `", name, "` is not register");
}

static bool match_prefix(psl::string_view base, psl::string_view candidate) {
  if (candidate.size() < base.size())
    return false;
  for (size_t i = 0; i < base.size(); i++)
    if (base[i] != candidate[i])
      return false;
  return true;
}

psl::string Context::complete(psl::string part) const {
  auto result = psl::string();

  for (const auto& f : functions_map) {
    if (match_prefix(part, f.first)) {
      if (result == "")
        result = f.first;

      auto i = part.size();
      for (;; i++) {
        if (i >= result.size())
          break;
        if (i >= f.first.size())
          break;
        if (result[i] != f.first[i])
          break;
      }
      result.resize(i);
    }
  }

  if (result.size() == 0)
    return result;
  return result.substr(part.size());
}

}  // namespace pine
