#pragma once
#include <string>

namespace pine {

inline std::string toString(std::string x) { return std::string(x); }
inline std::string toString(std::string_view x) { return std::string(x); }
inline std::string toString(const char* x) { return std::string(x); }

template <typename T>
requires requires(T x) { std::to_string(x); }
std::string toString(T x) { return std::to_string(x); }

template <typename T>
requires requires(T x) {
  std::size(x);
  x[0];
}
std::string toString(const T& x) {
  auto str = std::string("[");
  for (size_t i = 0; i < std::size(x); ++i) str += toString(x[i]) + ' ';
  str.back() = ']';
  return str;
}

template <typename... Ts>
requires(sizeof...(Ts) > 1) std::string toString(const Ts&... args) {
  return (toString(args) + ...);
}

}  // namespace pine
