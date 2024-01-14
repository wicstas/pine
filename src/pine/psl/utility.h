#pragma once

#include <pine/psl/type_traits.h>

namespace psl {

template <typename T>
T&& move(T& x) {
  return static_cast<T&&>(x);
}

// `TypeIdentityT` is used to enforce user provides a template argument
template <typename T>
T&& forward(TypeIdentityT<T>& x) {
  return static_cast<T&&>(x);
}

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

template <typename T, typename... Ts>
struct tuple {
  tuple(T value, Ts... rest) : value{psl::forward<T>(value)}, rest{psl::forward<Ts>(rest)...} {
  }
  T value;
  tuple<Ts...> rest;
};

template <typename T>
struct tuple<T> {
  tuple(T value) : value{psl::forward<T>(value)} {
  }
  T value;
};

template <typename T, typename F, typename... Ts, typename... Us>
auto apply(tuple<T, Ts...> t, F&& f, Us... args) {
  if constexpr (sizeof...(Ts) != 0)
    return apply(psl::move(t.rest), f, psl::forward<Us>(args)..., psl::move(t).value);
  else
    return f(psl::forward<Us>(args)..., psl::move(t).value);
}

template <typename T, typename F, typename... Ts, typename... Us>
auto apply(tuple<T, Ts...>& t, F&& f, Us&... args) {
  if constexpr (sizeof...(Ts) != 0)
    return apply(t.rest, f, args..., t.value);
  else
    return f(args..., t.value);
}

}  // namespace psl
