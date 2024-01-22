#pragma once

#include <pine/psl/string.h>

#include <typeinfo>

namespace psl {

constexpr size_t hash_combine(size_t seed, size_t value) {
  return seed ^ (value + 0x9e3779b9 + (seed << 6) + (seed >> 2));
}

template <typename T>
size_t type_id() {
  static auto id = typeid(T).hash_code();
  return id;
}

struct TypeId {
  TypeId() = default;
  TypeId(size_t code, bool IsConst, bool IsReference)
      : code{code}, IsConst{IsConst}, IsReference{IsReference} {};

  bool operator<(TypeId rhs) const {
    return code < rhs.code;
  }

  size_t code = invalid;
  bool IsConst = false;
  bool IsReference = false;
  static constexpr size_t invalid = static_cast<size_t>(-1);
};
template <typename T>
TypeId type_id_full() {
  static auto id = TypeId{typeid(T).hash_code(), psl::IsConst<T>,
                          psl::IsReference<T> || psl::IsRvReference<T>};
  return id;
}

string demangle(const char* mangled_type_name);

template <typename T>
const psl::string& type_name() {
  static psl::string str = demangle(typeid(T).name());
  return str;
}

void abort();

class string stacktrace();

}  // namespace psl
