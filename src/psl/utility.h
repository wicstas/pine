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
#define MOVE(x) psl::move(x)

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

template <typename T, typename... Args>
void initialize_with(T& x, Args&&... args) {
  x = T(FWD(args)...);
}

template <typename T, typename U>
struct pair {
  pair() = default;
  pair(T first, U second) : first{psl::forward<T>(first)}, second{psl::forward<U>(second)} {
  }

  T first;
  U second;
};

template <typename T, typename U>
pair<T, U> make_pair(T a, U b) {
  return pair<T, U>(static_cast<T>(a), static_cast<U>(b));
}

constexpr auto first_of_pair = [](auto&& pair) -> decltype(auto) { return FWD(pair).first; };
constexpr auto second_of_pair = [](auto&& pair) -> decltype(auto) { return FWD(pair).second; };

template <typename... Ts>
struct tuple;

template <typename T0>
struct tuple<T0> {
  tuple(T0 v0) : v0(FWD(v0)) {
  }
  template <typename U0>
  tuple& operator=(const tuple<U0>& rhs) {
    v0 = rhs.v0;
    return *this;
  }
  T0 v0;
};
template <typename T0, typename T1>
struct tuple<T0, T1> {
  tuple(T0 v0, T1 v1) : v0(FWD(v0)), v1(FWD(v1)) {
  }
  template <typename U0, typename U1>
  tuple& operator=(const tuple<U0, U1>& rhs) {
    v0 = rhs.v0;
    v1 = rhs.v1;
    return *this;
  }
  T0 v0;
  T1 v1;
};
template <typename T0, typename T1, typename T2>
struct tuple<T0, T1, T2> {
  tuple(T0 v0, T1 v1, T2 v2) : v0(FWD(v0)), v1(FWD(v1)), v2(FWD(v2)) {
  }
  template <typename U0, typename U1, typename U2>
  tuple& operator=(const tuple<U0, U1, U2>& rhs) {
    v0 = rhs.v0;
    v1 = rhs.v1;
    v2 = rhs.v2;
    return *this;
  }
  T0 v0;
  T1 v1;
  T2 v2;
};

template <typename... Ts>
auto make_tuple(Ts... xs) {
  return tuple<Ts...>(MOVE(xs)...);
}
template <typename... Ts>
auto tie(Ts&... xs) {
  return tuple<Ts&...>(xs...);
}

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
