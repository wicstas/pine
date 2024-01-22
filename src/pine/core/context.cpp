#include <pine/core/context.h>

namespace pine {

psl::map<size_t, psl::shared_ptr<TypeConcept>> types;

const Function* function_overload_resolution(const psl::multimap<psl::string, Function>& functions,
                                             psl::string_view name,
                                             const psl::vector<Variable>& args) {
  auto range = functions.equal_range(name);
  auto first = range.first;
  auto last = range.second;
  auto best_difference = args.size();
  auto best_match = first;
  auto any_match = false;

  for (auto fi = first; fi != last; fi++) {
    if (fi->second.is_internal_function())
      return &fi->second;
    const auto& arg_type_ids = fi->second.arg_type_ids();
    if (arg_type_ids.size() == args.size()) {
      auto difference = size_t{0};
      auto match = true;
      for (size_t i = 0; i < args.size(); i++) {
        if (args[i].type_id() == arg_type_ids[i].code ||
            arg_type_ids[i].code == psl::type_id<Variable>()) {
        } else if (!arg_type_ids[i].IsReference && args[i].convertible_to(arg_type_ids[i].code)) {
          difference += 1;
        } else {
          match = false;
          break;
        }
      }
      if (match && difference <= best_difference) {
        best_difference = difference;
        best_match = fi;
        any_match = true;
      }
    }
  }

  if (any_match)
    return &best_match->second;
  else
    return decltype(&best_match->second)(nullptr);
}

Variable* Context::find(psl::string_view name) {
  for (int i = int(variableStack.size()) - 1; i >= 0; i--) {
    if (auto it = variableStack[i].find(name); it != variableStack[i].end())
      return &it->second;
  }
  return nullptr;
}
Variable& Context::operator[](psl::string_view name) {
  for (int i = int(variableStack.size()) - 1; i >= 0; i--)
    if (auto it = variableStack[i].find(name); it != variableStack[i].end())
      return it->second;
  exception("Unable to find variable `", name, '`');
  PINE_UNREACHABLE;
}
Function Context::f(psl::string_view name, const psl::vector<Variable>& args) {
  if (auto func_ptr = function_overload_resolution(functions, name, args); func_ptr)
    return *func_ptr;

  auto arg_type_aliases =
      psl::space_by(psl::to<psl::vector<psl::string>>(
                        psl::transform(args, [](const Variable& arg) { return arg.type_alias(); })),
                    " ");
  auto candidates = psl::string{};
  auto range = functions.equal_range(name);
  for (auto it = range.first; it != range.second; it++) {
    candidates += name + "(" + psl::space_by(it->second.arg_type_aliases(), " ") + ")\n";
  }
  if (candidates.size() != 0)
    candidates.pop_back();

  if (candidates.size() != 0)
    exception("Unable to find function `", name, "` that accepts arguments of types `",
              arg_type_aliases, "`, \ncandidate functions are:\n", candidates);
  else {
    exception("Unable to find function `", name, "` that accepts arguments of types `",
              arg_type_aliases, "`, \nno function with this name is found");
  }
  PINE_UNREACHABLE;
}

psl::map<psl::string, Variable>& Context::top() {
  CHECK(variableStack.size());
  return variableStack.back();
}

psl::vector<psl::string> split(psl::string_view input, auto pred) {
  auto parts = psl::vector<psl::string>{};
  auto start = input.begin();
  while (true) {
    auto end = psl::find_if(psl::range(start, input.end()), pred);
    parts.push_back(psl::string{start, end});
    if (end == input.end())
      break;
    else
      start = psl::next(end);
  }

  return parts;
}

SourceLines::SourceLines(psl::string_view tokens, size_t lines_padding)
    : lines_padding{lines_padding} {
  lines = split(tokens, psl::isnewline);
  for (size_t i = 0; i < lines_padding; i++)
    lines.push_front("");
  for (size_t i = 0; i < lines_padding; i++)
    lines.push_back("");
}

psl::optional<psl::string_view> SourceLines::next_line(size_t row) const {
  CHECK_LE(row + lines_padding, lines.size());
  if (row + lines_padding == lines.size())
    return psl::nullopt;
  return lines[row];
}

psl::optional<char> SourceLines::next(size_t row, size_t column) const {
  if (auto line = next_line(row)) {
    CHECK_LT(column, line->size());
    return (*line)[column];
  }
  return psl::nullopt;
}

void SourceLines::error_impl(size_t row, size_t column, psl::string_view message) const {
  CHECK(lines_padding != invalid);
  auto vicinity = psl::string{};
  for (size_t i = row - lines_padding; i <= row; i++)
    vicinity += " | " + lines[i] + "\n";
  vicinity += "  -" + psl::string(column, '-') + "^\n";
  for (size_t i = row + 1; i <= row + lines_padding; i++)
    vicinity += " | " + lines[i] + "\n";

  if (vicinity.size())
    vicinity.pop_back();

  Fatal(message, "\n", vicinity);
}

}  // namespace pine
