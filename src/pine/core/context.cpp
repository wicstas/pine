#include <pine/core/context.h>

#include <psl/algorithm.h>

namespace pine {

Context Context::context;

psl::string signature_from(const TypeTag& rtype, psl::span<const TypeTag> ptypes) {
  auto r = psl::string("(");
  for (const auto& ptype : ptypes)
    r += ptype.sig() + ", ";
  if (ptypes.size() != 0)
    r.pop_back(2);
  r += "): " + rtype.sig();
  return r;
}

Context::Context() {
  auto& context = *this;

  context.type<void>("void");
  context.type<bool>("bool");
  context.type<int, TypeClass::Float>("i32");
  context.type<uint8_t, TypeClass::Float>("u8");
  context.type<float, TypeClass::Float>("f32").ctor_variant<int>();
  context.type<size_t, TypeClass::Float>("u64");

  add_f("==", [](bool a, bool b) -> bool { return a == b; });
  add_f("&&", [](bool a, bool b) -> bool { return a && b; });
  add_f("||", [](bool a, bool b) -> bool { return a || b; });
  add_f("!=", [](bool a, bool b) -> bool { return a != b; });
  add_f("=", [](bool& a, bool b) -> bool& { return a = b; });

  context.type<int>().ctor_variant_explicit<float>();
  context("++x") = [](int& x) -> decltype(auto) { return ++x; };
  context("--x") = [](int& x) -> decltype(auto) { return --x; };
  context("x++") = [](int& x) -> decltype(auto) { return x++; };
  context("x--") = [](int& x) -> decltype(auto) { return x--; };
  context("%") = [](int a, int b) { return a % b; };
  context("%=") = [](int& a, int b) -> int& { return a %= b; };
  context("+") = [](int a, float b) { return a + b; };
  context("-") = [](int a, float b) { return a - b; };
  context("*") = [](int a, float b) { return a * b; };
  context("/") = [](int a, float b) { return a / b; };
  context("+") = [](float a, int b) { return a + b; };
  context("-") = [](float a, int b) { return a - b; };
  context("*") = [](float a, int b) { return a * b; };
  context("/") = [](float a, int b) { return a / b; };
  context("+=") = [](int& a, float b) -> int& { return a += b; };
  context("-=") = [](int& a, float b) -> int& { return a -= b; };
  context("*=") = [](int& a, float b) -> int& { return a *= b; };
  context("/=") = [](int& a, float b) -> int& { return a /= b; };
  context("+=") = [](float& a, int b) -> float& { return a += b; };
  context("-=") = [](float& a, int b) -> float& { return a -= b; };
  context("*=") = [](float& a, int b) -> float& { return a *= b; };
  context("/=") = [](float& a, int b) -> float& { return a /= b; };
  context("^") = [](int a, int b) { return psl::powi(a, b); };
  context("^") = [](float a, float b) { return psl::pow(a, b); };

  context.type<char*>("char*");
  context.type<const char*>("cstr");
  context.type<psl::default_allocator<char>>("default_allocator");
  context.type<psl::string_allocator<char>>("string_allocator");
  context.type<psl::vector<char>>("vector")
      .layout<char*, size_t, size_t, psl::default_allocator<char>>();
  context.type<psl::string>("str")
      .layout<char*, size_t, size_t, psl::string_allocator<char>>()
      .ctor<const char*>()
      .method<&psl::string::size>("size")
      .ctor_variant<[](auto x) { return psl::to_string(FWD(x)); }, bool, uint8_t, int, float,
                    size_t>();
  add_f("=", [](psl::string& a, psl::string b) -> psl::string& { return a = b; });
  add_f("+=", [](psl::string& a, psl::string b) -> psl::string& { return a += b; });
  add_f("+", [](psl::string a, psl::string b) { return a + b; });
  context.type<psl::string_view>("str_view")
      .layout<const char*, size_t>()
      .ctor_variant<const psl::string&>();

  context.type<void*>("void*");
  context("malloc") = [](size_t size) { return malloc(psl::max(size, size_t(1))); };

//   struct FunctionObject {
    //     FunctionObject(void* obj, void* f) : obj(obj), f(f) {
    //     }
    //     FunctionObject(const FunctionObject&) = delete;
    //     FunctionObject& operator=(const FunctionObject&) = delete;
    //     FunctionObject(FunctionObject&& rhs) : obj(psl::exchange(rhs.obj, nullptr)), f(rhs.f) {
    //     }
    //     FunctionObject& operator=(FunctionObject&& rhs) {
    //       obj = psl::exchange(rhs.obj, nullptr);
    //       f = rhs.f;
    //       return *this;
    //     }
    //     ~FunctionObject() {
    //       if (obj)
    //         psl::free(obj);
    //     }
//     void* obj;
//     void* f;
//   };
//   context.type<FunctionObject>("@FunctionObject");
}

// static int common_prefix_length(psl::string_view a, psl::string_view b) {
//   auto i = size_t(0);
//   for (; i < psl::min(a.size(), b.size()); i++) {
//     if (a[i] != b[i])
//       break;
//   }
//   return i;
// }

// static int common_part_length(psl::string_view a, psl::string_view b) {
//   auto max_len = 0;
//   for (size_t i = 0; i < b.size(); i++) {
//     max_len = psl::max(common_prefix_length(a, b.subview(i)), max_len);
//   }
//   return max_len;
// }

void Context::add_f(psl::string name, Function func) {
  auto [first, last] = function_name_to_index.equal_range(name);
  while (first != last) {
    const auto& [fname, fi] = *first;
    if (functions[fi].unique_name() == func.unique_name()) {
      function_name_to_index.erase(first);
      std::tie(first, last) = function_name_to_index.equal_range(name);
    } else {
      ++first;
    }
  }
  function_name_to_index.insert({MOVE(name), functions.size()});
  functions.push_back(MOVE(func));
}
Context::FindUniqueFResult Context::find_unique_f(psl::string_view name) const {
  auto count = function_name_to_index.count(name);
  if (count == 0)
    return {size_t(-1), FindUniqueFResult::FindNone};
  else if (count > 1)
    return {size_t(-1), FindUniqueFResult::FindTooMany};
  else
    return {function_name_to_index.find(name)->second, FindUniqueFResult::Success};
}
Context::FindFResult Context::find_f(psl::string_view name, psl::span<const TypeTag> atypes) const {
  auto [first, last] = function_name_to_index.equal_range(name);
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
    if (atypes.size() == ptypes.size()) {
      auto converts = psl::vector<Context::FindFResult::ArgumentConversion>();
      auto difference = size_t{0};
      for (size_t i = 0; i < atypes.size(); i++) {
        if (atypes[i].name == ptypes[i].name) {
        } else if ((!ptypes[i].is_ref || ptypes[i].is_const) &&
                   is_registered_type(ptypes[i].name)) {
          if (auto cr = find_unique_f("@convert." + atypes[i].name + "." + ptypes[i].name)) {
            converts.push_back({i, cr.fi, ptypes[i]});
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
    return {best_candidates[0], best_converts};
  } else if (best_candidates.size() > 1) {
    auto candidates = psl::string();
    for (auto fi : best_candidates)
      candidates += functions[fi].unique_name() + "\n";
    candidates.pop_back();
    Fatal("Ambiguous function call `", name, "(", arg_signature, ")`, candidates:\n", candidates);
  } else {
    if (first != last) {
      auto candidates = psl::string();
      for (auto [name, fi] : psl::range(first, last))
        candidates += functions[fi].unique_name() + "\n";
      Fatal("Function `", name, "(", arg_signature, ")` is not found, candidates:\n", candidates);
    } else {
      //   auto likely_func_name = psl::string();
      //   auto max_common_part_length = 0;
      //   for (const auto& f : function_name_to_index) {
      //     if (auto len = common_part_length(name, f.first); len > max_common_part_length) {
      //       likely_func_name = f.first;
      //       max_common_part_length = len;
      //     }
      //   }
      //   if (likely_func_name != "")
      //     Fatal("Function `", name, "(", arg_signature, ")` is not found, did you mean `",
      //           likely_func_name, "`?");
      //   else
      Fatal("Function `", name, "(", arg_signature, ")` is not found");
    }
  }
}

psl::string Context::type_name_from_id(size_t type_id) const {
  if (auto it = type_id_to_name.find(type_id); it != type_id_to_name.end())
    return it->second;
  else
    Fatal("Type with id `", type_id, "` is not registered");
}
TypeTrait* Context::create_type_trait(psl::string name, size_t byte_size) {
  if (auto it = types.find(name); it != types.end())
    Fatal("Type `", name, "` already exists");
  auto trait =
      types.insert({name, psl::make_unique<TypeTrait>(name, byte_size)}).first->second.get();
  return trait;
}
TypeTrait* Context::get_type_trait(psl::string_view name) {
  if (auto it = types.find(name); it != types.end())
    return it->second.get();
  else
    Fatal("Type `", name, "` is not registered");
}
const TypeTrait* Context::get_type_trait(psl::string_view name) const {
  if (auto it = types.find(name); it != types.end())
    return it->second.get();
  else
    Fatal("Type `", name, "` is not registered");
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

  if (part == "")
    return result;

  for (const auto& f : function_name_to_index) {
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
