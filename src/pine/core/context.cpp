#include <pine/core/context.h>

#include <psl/algorithm.h>

namespace pine {

struct Any {};

Context::Context() {
  auto& context = *this;
  context.type<Any>("any");
  context.type<Variable>("variable");
  context("=") = low_level(
      [](psl::span<const Variable*> args) {
        args[0]->as<Variable&>() = *args[1];
        return psl::Empty();
      },
      psl::type_id<void>(),
      psl::vector_of(psl::TypeId(psl::type_id<Variable>(), false, true),
                     psl::TypeId(psl::type_id<Any>(), false, false)));
  context("__typename") = low_level(
      [](psl::span<const Variable*> args) -> psl::string { return args[0]->type_name(); },
      psl::type_id<psl::string>(), psl::vector_of(psl::TypeId(psl::type_id<Any>(), false, false)));
  context.type<size_t>("u64");
  context.type<psl::Empty>("void");
  context.type<void>("void");
  context.type<Function>("function");
  context.type<int32_t, Context::Float>("i32").ctor_variant<float>(true);
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

  context.type<float, Context::Float>("f32").ctor_variant<int32_t>();
  context("^") = +[](float a, float b) { return psl::pow(a, b); };

  context.type<bool>("bool");
  add_f(
      "==", +[](bool a, bool b) -> bool { return a == b; });
  add_f(
      "!=", +[](bool a, bool b) -> bool { return a != b; });
  add_f(
      "=", +[](bool& a, bool b) -> bool& { return a = b; });
  context.type<bool>("bool");

  context.type<psl::string>("str");
  context("=") = +[](psl::string& a, psl::string b) -> psl::string& { return a = b; };
  context("+=") = +[](psl::string& a, psl::string b) -> psl::string& { return a += b; };
  context("+") = +[](psl::string a, psl::string b) { return a + b; };
}

size_t Context::TypeTrait::find_member_accessor_index(psl::string_view member_name) const {
  if (auto it = member_accessors.find(member_name); it != member_accessors.end())
    return it->second;
  else
    return size_t(-1);
}
size_t Context::TypeTrait::find_convert_to_index(size_t to_type_id) const {
  if (auto it = convert_tos.find(to_type_id); it != convert_tos.end())
    return it->second;
  else
    return size_t(-1);
}
size_t Context::TypeTrait::find_convert_from_index(size_t from_type_id) const {
  if (auto it = convert_froms.find(from_type_id); it != convert_froms.end())
    return it->second;
  else
    return size_t(-1);
}

size_t Context::converter_index(size_t from_id, size_t to_id) const {
  auto index = size_t(-1);
  if (auto it = types.find(from_id); it != types.end())
    index = it->second.find_convert_to_index(to_id);

  if (index == size_t(-1))
    if (auto it = types.find(to_id); it != types.end())
      index = it->second.find_convert_from_index(from_id);

  return index;
}

// static bool is_id_part(char c) {
//   return psl::isalpha(c) || psl::isdigit(c) || c == '_';
// }

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

static psl::string function_signature(const Context& context, const Function& function) {
  auto sig = psl::string("(");
  for (const auto& ptid : function.parameter_type_ids()) {
    sig += context.get_type(ptid.code).alias + ", ";
  }
  if (sig.size() != 0) {
    sig.pop_back();
    sig.pop_back();
  }
  sig += "): " + context.get_type(function.return_type_id()).alias;
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
        result.candidates += name + function_signature(*this, functions[fi]) + "\n";
      result.candidates.pop_back();
    }
  }

  return result;
}

Context::FindFResult Context::find_f(psl::string_view name, psl::span<size_t> atids) const {
  auto [first, last] = functions_map.equal_range(name);
  auto best_coeff = atids.size();
  auto best_candidates = psl::vector<size_t>();
  auto best_converts = psl::vector<Context::FindFResult::ArgumentConversion>();

  auto arg_signature = psl::string();
  for (auto atid : atids)
    arg_signature += get_type(atid).alias + ", ";
  if (arg_signature.size()) {
    arg_signature.pop_back();
    arg_signature.pop_back();
  }

  for (auto fi : psl::range(first, last)) {
    const auto& ptids = functions[fi.second].parameter_type_ids();
    if (ptids.size() == 1 && ptids[0].code == psl::type_id<psl::span<const Variable*>>()) {
      best_converts = {};
      best_candidates.clear();
      best_candidates.push_back(fi.second);
      break;
    } else if (atids.size() == ptids.size()) {
      auto converts = psl::vector<Context::FindFResult::ArgumentConversion>();
      auto difference = size_t{0};
      for (size_t i = 0; i < atids.size(); i++) {
        if (atids[i] == ptids[i].code) {
        } else if (ptids[i].code == psl::type_id<Any>()) {
        } else if (auto idx = converter_index(atids[i], ptids[i].code);
                   (!ptids[i].is_ref || ptids[i].is_const) && idx != size_t(-1)) {
          converts.push_back({i, idx, ptids[i].code});
          difference += 1;
        } else {
          difference = atids.size() + 1;
          break;
        }
      }
      if (difference <= best_coeff) {
        if (difference < best_coeff) {
          best_candidates.clear();
        }
        best_coeff = difference;
        best_candidates.push_back(fi.second);
        best_converts = converts;
      }
    }
  }

  if (best_candidates.size() == 1) {
    auto fi = best_candidates.front();
    return {fi, functions[fi].return_type_id(), best_converts};
  } else if (best_candidates.size() > 1) {
    auto candidates = psl::string();
    for (auto fi : best_candidates)
      candidates += name + function_signature(*this, functions[fi]) + "\n";
    if (candidates.size())
      candidates.pop_back();
    exception("Ambiguous function call `", name, "(", arg_signature, ")`, candidates:\n",
              candidates);
  } else {
    if (first != last) {
      auto candidates = psl::string();
      for (auto [name, fi] : psl::range(first, last))
        candidates += name + function_signature(*this, functions[fi]) + "\n";
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
        exception("Function `", name, "` is not found, did you mean `", likely_func_name, "`?");
      else
        exception("Function `", name, "` is not found");
    }
  }
}

void Context::add_f(psl::string name, Function func) {
  functions_map.insert({psl::move(name), functions.size()});
  functions.push_back(psl::move(func));
}

Variable Context::call(psl::string_view name, psl::span<const Variable*> args) const {
  auto atids =
      psl::to<psl::vector<size_t>>(psl::transform(args, [](auto x) { return x->type_id(); }));
  auto fr = find_f(name, atids);
  CHECK(fr.converts.size() == 0);
  return functions[fr.function_index].call(args);
}

size_t Context::find_variable(psl::string_view name) const {
  if (auto it = variables_map.find(name); it != variables_map.end())
    return it->second;
  else
    return size_t(-1);
}
void Context::add_variable(psl::string name, Variable var) {
  variables_map.insert({psl::move(name), variables.size()});
  variables.push_back(psl::move(var));
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

size_t Context::find_type_id(psl::string_view name) const {
  for (const auto& type : types)
    if (type.second.alias == name)
      return type.first;
  return size_t(-1);
}
size_t Context::get_type_id(psl::string_view name) const {
  if (auto id = find_type_id(name); id != size_t(-1))
    return id;
  exception("Type `", name, "` is not registered");
}

Context::TypeTrait& Context::get_type(size_t type_id) {
  if (auto it = types.find(type_id); it != types.end())
    return it->second;
  else
    exception("Type with id ", type_id, " is not register");
}
const Context::TypeTrait& Context::get_type(size_t type_id) const {
  if (auto it = types.find(type_id); it != types.end())
    return it->second;
  else
    exception("Type with id ", type_id, " is not register");
}

}  // namespace pine
