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

#define FWD(x) static_cast<decltype(x)&&>(x)
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

#pragma pack(push, 1)
template <typename... Ts>
struct tuple;

template <>
struct tuple<> {
  tuple() {
  }
  tuple& operator=(const tuple<>&) {
    return *this;
  }
};
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
template <typename T0, typename T1, typename T2, typename T3>
struct tuple<T0, T1, T2, T3> {
  tuple(T0 v0, T1 v1, T2 v2, T3 v3) : v0(FWD(v0)), v1(FWD(v1)), v2(FWD(v2)), v3(FWD(v3)) {
  }
  template <typename U0, typename U1, typename U2, typename U3>
  tuple& operator=(const tuple<U0, U1, U2, U3>& rhs) {
    v0 = rhs.v0;
    v1 = rhs.v1;
    v2 = rhs.v2;
    v3 = rhs.v3;
    return *this;
  }
  T0 v0;
  T1 v1;
  T2 v2;
  T3 v3;
};
template <typename T0, typename T1, typename T2, typename T3, typename T4>
struct tuple<T0, T1, T2, T3, T4> {
  tuple(T0 v0, T1 v1, T2 v2, T3 v3, T4 v4)
      : v0(FWD(v0)), v1(FWD(v1)), v2(FWD(v2)), v3(FWD(v3)), v4(FWD(v4)) {
  }
  template <typename U0, typename U1, typename U2, typename U3, typename U4>
  tuple& operator=(const tuple<U0, U1, U2, U3, U4>& rhs) {
    v0 = rhs.v0;
    v1 = rhs.v1;
    v2 = rhs.v2;
    v3 = rhs.v3;
    v4 = rhs.v4;
    return *this;
  }
  T0 v0;
  T1 v1;
  T2 v2;
  T3 v3;
  T4 v4;
};

template <typename T0, typename T1, typename T2, typename T3, typename T4, typename T5>
struct tuple<T0, T1, T2, T3, T4, T5> {
  tuple(T0 v0, T1 v1, T2 v2, T3 v3, T4 v4, T5 v5)
      : v0(FWD(v0)), v1(FWD(v1)), v2(FWD(v2)), v3(FWD(v3)), v4(FWD(v4)), v5(FWD(v5)) {
  }
  template <typename U0, typename U1, typename U2, typename U3, typename U4, typename U5>
  tuple& operator=(const tuple<U0, U1, U2, U3, U4, U5>& rhs) {
    v0 = rhs.v0;
    v1 = rhs.v1;
    v2 = rhs.v2;
    v3 = rhs.v3;
    v4 = rhs.v4;
    v5 = rhs.v5;
    return *this;
  }
  T0 v0;
  T1 v1;
  T2 v2;
  T3 v3;
  T4 v4;
  T5 v5;
};
template <typename T0, typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
struct tuple<T0, T1, T2, T3, T4, T5, T6> {
  tuple(T0 v0, T1 v1, T2 v2, T3 v3, T4 v4, T5 v5, T6 v6)
      : v0(FWD(v0)), v1(FWD(v1)), v2(FWD(v2)), v3(FWD(v3)), v4(FWD(v4)), v5(FWD(v5)), v6(FWD(v6)) {
  }
  template <typename U0, typename U1, typename U2, typename U3, typename U4, typename U5,
            typename U6>
  tuple& operator=(const tuple<U0, U1, U2, U3, U4, U5, U6>& rhs) {
    v0 = rhs.v0;
    v1 = rhs.v1;
    v2 = rhs.v2;
    v3 = rhs.v3;
    v4 = rhs.v4;
    v5 = rhs.v5;
    v6 = rhs.v6;
    return *this;
  }
  T0 v0;
  T1 v1;
  T2 v2;
  T3 v3;
  T4 v4;
  T5 v5;
  T6 v6;
};
template <typename T0, typename T1, typename T2, typename T3, typename T4, typename T5, typename T6,
          typename T7>
struct tuple<T0, T1, T2, T3, T4, T5, T6, T7> {
  tuple(T0 v0, T1 v1, T2 v2, T3 v3, T4 v4, T5 v5, T6 v6, T7 v7)
      : v0(FWD(v0)),
        v1(FWD(v1)),
        v2(FWD(v2)),
        v3(FWD(v3)),
        v4(FWD(v4)),
        v5(FWD(v5)),
        v6(FWD(v6)),
        v7(FWD(v7)) {
  }
  template <typename U0, typename U1, typename U2, typename U3, typename U4, typename U5,
            typename U6, typename U7>
  tuple& operator=(const tuple<U0, U1, U2, U3, U4, U5, U6, U7>& rhs) {
    v0 = rhs.v0;
    v1 = rhs.v1;
    v2 = rhs.v2;
    v3 = rhs.v3;
    v4 = rhs.v4;
    v5 = rhs.v5;
    v6 = rhs.v6;
    v7 = rhs.v7;
    return *this;
  }
  T0 v0;
  T1 v1;
  T2 v2;
  T3 v3;
  T4 v4;
  T5 v5;
  T6 v6;
  T7 v7;
};
#pragma pack(pop)

template <typename... Ts>
auto make_tuple(Ts... xs) {
  return tuple<Ts...>(MOVE(xs)...);
}
template <typename... Ts>
auto tie(Ts&... xs) {
  return tuple<Ts&...>(xs...);
}

template <typename T>
constexpr int tuple_size = 0;
template <typename... Args>
constexpr int tuple_size<tuple<Args...>> = sizeof...(Args);
template <typename... Args>
constexpr int tuple_size<tuple<Args...>&> = sizeof...(Args);

template <typename T, typename F>
decltype(auto) apply(T&& t, F&& f) {
  constexpr auto size = tuple_size<T>;
  if constexpr (size == 0)
    return f();
  else if constexpr (size == 1)
    return f(t.v0);
  else if constexpr (size == 2)
    return f(t.v0, t.v1);
  else if constexpr (size == 3)
    return f(t.v0, t.v1, t.v2);
  else if constexpr (size == 4)
    return f(t.v0, t.v1, t.v2, t.v3);
  else if constexpr (size == 5)
    return f(t.v0, t.v1, t.v2, t.v3, t.v4);
  else if constexpr (size == 6)
    return f(t.v0, t.v1, t.v2, t.v3, t.v4, t.v5);
  else if constexpr (size == 7)
    return f(t.v0, t.v1, t.v2, t.v3, t.v4, t.v5, t.v6);
  else if constexpr (size == 8)
    return f(t.v0, t.v1, t.v2, t.v3, t.v4, t.v5, t.v6, t.v7);
}

template <typename T, typename F>
decltype(auto) apply_with_index(T&& t, F&& f) {
  constexpr auto size = tuple_size<T>;
  if constexpr (size == 0)
    return f();
  else if constexpr (size == 1)
    return f(psl::make_pair(t.v0, 0));
  else if constexpr (size == 2)
    return f(psl::make_pair(t.v0, 0), psl::make_pair(t.v1, 1));
  else if constexpr (size == 3)
    return f(psl::make_pair(t.v0, 0), psl::make_pair(t.v1, 1), psl::make_pair(t.v2, 2));
  else if constexpr (size == 4)
    return f(psl::make_pair(t.v0, 0), psl::make_pair(t.v1, 1), psl::make_pair(t.v2, 2),
             psl::make_pair(t.v3, 3));
  else if constexpr (size == 5)
    return f(psl::make_pair(t.v0, 0), psl::make_pair(t.v1, 1), psl::make_pair(t.v2, 2),
             psl::make_pair(t.v3, 3), psl::make_pair(t.v4, 4));
  else if constexpr (size == 6)
    return f(psl::make_pair(t.v0, 0), psl::make_pair(t.v1, 1), psl::make_pair(t.v2, 2),
             psl::make_pair(t.v3, 3), psl::make_pair(t.v4, 4), psl::make_pair(t.v5, 5));
  else if constexpr (size == 7)
    return f(psl::make_pair(t.v0, 0), psl::make_pair(t.v1, 1), psl::make_pair(t.v2, 2),
             psl::make_pair(t.v3, 3), psl::make_pair(t.v4, 4), psl::make_pair(t.v5, 5),
             psl::make_pair(t.v6, 6));
  else if constexpr (size == 8)
    return f(psl::make_pair(t.v0, 0), psl::make_pair(t.v1, 1), psl::make_pair(t.v2, 2),
             psl::make_pair(t.v3, 3), psl::make_pair(t.v4, 4), psl::make_pair(t.v5, 5),
             psl::make_pair(t.v6, 6), psl::make_pair(t.v7, 7));
}

template <typename... Fs>
auto overloaded_lambda(Fs... fs) {
  struct Overloads : Fs... {
    using Fs::operator()...;
  };
  return Overloads{fs...};
}

}  // namespace psl
