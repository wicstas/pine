#include <pine/core/context.h>

#include <pine/psl/algorithm.h>

namespace pine {

Context::Context() {
  auto& context = *this;
  context.type<int, Context::Float>("int").to<float>().ctor_variant<float>(true);
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

  context.type<float, Context::Float>("float");
  context("^") = +[](float a, float b) { return psl::pow(a, b); };

  context.type<bool>("bool");
  add_f(
      "==", +[](bool a, bool b) -> bool { return a == b; });
  add_f(
      "!=", +[](bool a, bool b) -> bool { return a != b; });
  add_f(
      "=", +[](bool& a, bool b) -> bool& { return a = b; });
  context.type<bool>("bool");

  context.type<psl::string>("string");
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

static bool is_id_part(char c) {
  return psl::isalpha(c) || psl::isdigit(c) || c == '_';
}

static int difference(psl::string_view a, psl::string_view b) {
  auto diff = 0;
  for (size_t i = 0; i < psl::max(a.size(), b.size()); i++) {
    if (i >= a.size())
      diff++;
    else if (i >= b.size())
      diff++;
    else if (a[i] != b[i]) {
      if (is_id_part(a[i]) && !is_id_part(b[i]))
        diff += 1000;
      if (!is_id_part(a[i]) && is_id_part(b[i]))
        diff += 1000;
      diff += 8;
    }
  }
  return diff;
}

Context::FindFResult Context::find_f(psl::string_view name, psl::span<size_t> arg_type_ids) const {
  auto [first, last] = functions_map.equal_range(name);
  auto min_difference = size_t(-1);
  auto best_match = last;
  auto best_converts = psl::vector<Context::FindFResult::ArgumentConversion>();

  for (auto fi = first; fi != last; fi++) {
    const auto& param_type_ids = functions[fi->second].parameter_type_ids();
    if (param_type_ids.size() == 1 &&
        param_type_ids[0].code == psl::type_id<psl::span<const Variable*>>()) {
      best_converts = {};
      best_match = fi;
      break;
    } else if (arg_type_ids.size() == param_type_ids.size()) {
      auto converts = psl::vector<Context::FindFResult::ArgumentConversion>();
      auto difference = size_t{0};
      auto match = true;
      for (size_t i = 0; i < arg_type_ids.size(); i++) {
        if (param_type_ids[i].code == psl::type_id<Variable>()) {
        } else if (arg_type_ids[i] == param_type_ids[i].code) {
        } else if (auto idx = converter_index(arg_type_ids[i], param_type_ids[i].code);
                   !param_type_ids[i].is_ref && idx != size_t(-1)) {
          converts.push_back({i, idx, param_type_ids[i].code});
          difference += 1;
        } else {
          match = false;
          break;
        }
      }
      if (match && difference <= min_difference) {
        min_difference = difference;
        best_match = fi;
        best_converts = converts;
      }
    }
  }

  if (best_match != last) {
    auto fi = best_match->second;
    return {fi, functions[fi].return_type_id(), best_converts};
  } else {
    if (first != last) {
      auto arg_type_aliases = psl::string();
      auto candidates = psl::string();
      for (auto arg_type_id : arg_type_ids) {
        if (auto it = types.find(arg_type_id); it != types.end())
          arg_type_aliases += it->second.alias + ", ";
        else
          Fatal("Some type trait is not set");
      }
      if (arg_type_aliases.size()) {
        arg_type_aliases.pop_back();
        arg_type_aliases.pop_back();
      }
      for (auto fi = first; fi != last; fi++) {
        candidates += fi->first + '(';
        for (auto param_type_id : functions[fi->second].parameter_type_ids()) {
          if (auto it = types.find(param_type_id.code); it != types.end())
            candidates += it->second.alias + ", ";
          else
            Fatal("Some type trait is not set");
        }
        if (functions[fi->second].n_parameters() != 0) {
          candidates.pop_back();
          candidates.pop_back();
        }
        candidates += ")\n";
      }
      if (candidates.size())
        candidates.pop_back();
      exception("No function `", name, "` that accepts `", arg_type_aliases,
                "` is found, candidates:\n", candidates);
    } else {
      auto likely_func_name = psl::string();
      auto min_difference = 8;
      for (const auto& f : functions_map) {
        if (f.first == "x++")
          continue;
        if (f.first == "x--")
          continue;
        if (auto diff = difference(name, f.first); diff < min_difference) {
          likely_func_name = f.first;
          min_difference = diff;
        }
      }

      if (likely_func_name != "")
        exception("No function named `", name, "` is found, did you mean `", likely_func_name,
                  "`?");
      else
        exception("No function named `", name, "` is found");
    }
  }
}

void Context::add_f(psl::string name, Function func) {
  functions_map.insert({psl::move(name), functions.size()});
  functions.push_back(psl::move(func));
}

Variable Context::call(psl::string_view name, psl::span<const Variable*> args) const {
  auto arg_type_ids = psl::to<psl::vector<size_t>>(
      psl::transform(args, [](const Variable* x) { return x->type_id(); }));
  auto fr = find_f(name, arg_type_ids);
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

psl::vector<psl::string> Context::candidates(psl::string part) const {
  auto result = psl::vector<psl::string>();
  for (const auto& f : functions_map) {
    if (match_prefix(part, f.first)) {
      auto signature = f.first + "(";
      for (auto param_type_id : functions[f.second].parameter_type_ids()) {
        if (auto it = types.find(param_type_id.code); it != types.end())
          signature += it->second.alias + ", ";
        else
          Fatal("Some type trait is not set");
      }
      if (functions[f.second].n_parameters() != 0) {
        signature.pop_back();
        signature.pop_back();
      }
      result.push_back(signature + ")");
    }
  }

  return result;
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
