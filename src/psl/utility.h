#pragma once

#include <psl/type_traits.h>

namespace psl {

template <typename T>
T&& move(T& x) {
  return static_cast<T&&>(x);
}

// `TypeIdentity` is used to enforce user to provide template argument `T`
template <typename T>
T&& forward(TypeIdentity<T>& x) {
  return static_cast<T&&>(x);
}

#define FWD(x) psl::forward<decltype(x)>(x)

template <typename T>
void swap(T& x, T& y) {
  auto temp = psl::move(x);
  x = psl::move(y);
  y = psl::move(temp);
}

template <typename T, typename U>
T exchange(T& x, U y) {
  auto old = psl::move(x);
  x = psl::move(y);
  return old;
}

template <typename T, typename U>
struct pair {
  pair() = default;
  pair(T first, U second) : first{psl::forward<T>(first)}, second{psl::forward<U>(second)} {
  }

  T first;
  U second;
};

constexpr auto first_of_pair = [](auto&& pair) -> decltype(auto) { return FWD(pair).first; };
constexpr auto second_of_pair = [](auto&& pair) -> decltype(auto) { return FWD(pair).second; };

template <typename T, typename... Ts>
struct tuple {
  tuple(T value, Ts... rest) : value(static_cast<T&&>(value)), rest{static_cast<Ts&&>(rest)...} {
  }
  T value;
  tuple<Ts...> rest;
};

template <typename T>
struct tuple<T> {
  tuple(T value) : value(static_cast<T&&>(value)) {
  }
  T value;
};

template <typename T, typename F, typename... Ts, typename... Us>
auto apply(tuple<T, Ts...> t, F&& f, Us... args) {
  if constexpr (sizeof...(Ts) != 0)
    return apply(psl::move(t.rest), f, static_cast<Us&&>(args)..., psl::move(t).value);
  else
    return f(static_cast<Us&&>(args)..., psl::move(t).value);
}

template <typename T, typename F, typename... Ts, typename... Us>
auto apply(tuple<T, Ts...>& t, F&& f, Us&... args) {
  if constexpr (sizeof...(Ts) != 0)
    return apply(t.rest, f, args..., t.value);
  else
    return f(args..., t.value);
}

}  // namespace psl
