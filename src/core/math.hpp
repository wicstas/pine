#pragma once
#include <core/string.hpp>
#include <core/check.hpp>

#include <cstdint>
#include <cmath>
#include <array>

namespace pine {

auto sqr(auto x) { return x * x; }
auto lengthSquared(auto x) { return sqr(x); }
auto length(auto x) { return std::sqrt(lengthSquared(x)); }

#define CHECK_EQF(a, b)                                            \
  do                                                               \
    if (length(a - b) > 1e-4f) {                                   \
      reportFailedCheck(toString("Check [" #a " ~= " #b            \
                                 "] failed, where\n  " #a "  =  ", \
                                 a, "\n  " #b "  =  ", b));        \
      abort();                                                     \
    }                                                              \
  while (false)

constexpr int sum(auto... xs) { return (xs + ...); }

template <int N, typename F>
constexpr auto dispatchSequence(F f) {
  return [&]<int... Is>(std::integer_sequence<int, Is...>) {
    return f.template operator()<Is...>();
  }
  (std::make_integer_sequence<int, N>());
}
template <int N, typename F>
constexpr void dispatchIndex(F f) {
  dispatchSequence<N>([&]<int... Is>() {
    const auto dump = [](auto...) {};
    dump((f.template operator()<Is>(), 0)...);
  });
}

template <typename A, typename B, char op>
struct ArithmeticResult_;
template <typename A, typename B>
  requires requires(A a, B b) { a + b; }
struct ArithmeticResult_<A, B, '+'> {
  using type = decltype(std::declval<A>() + std::declval<B>());
};
template <typename A, typename B>
  requires requires(A a, B b) { a - b; }
struct ArithmeticResult_<A, B, '-'> {
  using type = decltype(std::declval<A>() - std::declval<B>());
};
template <typename A, typename B>
  requires requires(A a, B b) { a* b; }
struct ArithmeticResult_<A, B, '*'> {
  using type = decltype(std::declval<A>() * std::declval<B>());
};
template <typename A, typename B>
  requires requires(A a, B b) { a / b; }
struct ArithmeticResult_<A, B, '/'> {
  using type = decltype(std::declval<A>() / std::declval<B>());
};
template <typename A, typename B, char op>
using ArithmeticResult = typename ArithmeticResult_<A, B, op>::type;

template <typename T, int N>
struct Vector;

template <typename Ref, typename T>
constexpr int dimensionOf = 1;
template <typename Ref, typename T, int N>
constexpr int dimensionOf<Ref, Vector<T, N>> = N;
template <typename T, int N>
constexpr int dimensionOf<Vector<T, N>, Vector<T, N>> = 1;

template <int N, typename T>
constexpr bool isVectorN = false;
template <int N, typename T>
constexpr bool isVectorN<N, Vector<T, N>> = true;

template <typename T, int N>
struct Vector {
  Vector() = default;

  template <typename X>
    requires(dimensionOf<T, X> == 1)
  Vector(X x) {
    dispatchIndex<N>([&]<int I>() { at(I) = x; });
  }
  template <typename X>
    requires(dimensionOf<T, X> > N)
  explicit Vector(X x) {
    dispatchIndex<N>([&]<int I>() { at(I) = x[I]; });
  }

  template <typename... Ts>
    requires(sum(dimensionOf<T, Ts>...) == N)
  Vector(Ts... xs) {
    constructFrom<0>(xs...);
  }

  constexpr std::size_t size() const { return N; }

  T& operator[](int i) { return at(i); }
  const T& operator[](int i) const { return at(i); }

  T& at(int i) {
    DCHECK_GE(i, 0);
    DCHECK_LT(i, N);
    return components[i];
  }
  const T& at(int i) const {
    DCHECK_GE(i, 0);
    DCHECK_LT(i, N);
    return components[i];
  }

  template <typename B>
  Vector operator+=(Vector<B, N> b) {
    dispatchSequence<N>([&]<int I>() { at(I) += b[I]; });
    return *this;
  }
  template <typename B>
  Vector operator-=(Vector<B, N> b) {
    dispatchSequence<N>([&]<int I>() { at(I) -= b[I]; });
    return *this;
  }
  template <typename B>
  Vector operator*=(Vector<B, N> b) {
    dispatchSequence<N>([&]<int I>() { at(I) *= b[I]; });
    return *this;
  }
  template <typename B>
  Vector operator/=(Vector<B, N> b) {
    dispatchSequence<N>([&]<int I>() { at(I) /= b[I]; });
    return *this;
  }
  template <typename B>
  Vector operator*=(B b) {
    dispatchSequence<N>([&]<int I>() { at(I) *= b; });
    return *this;
  }
  template <typename B>
  Vector operator/=(B b) {
    dispatchSequence<N>([&]<int I>() { at(I) /= b; });
    return *this;
  }

  friend auto operator<=>(Vector, Vector) = default;

 private:
  template <int Offset, typename X, typename... Xs>
  void constructFrom(X x, Xs... xs) {
    constexpr int dim = dimensionOf<T, X>;

    if constexpr (dim > 1)
      dispatchIndex<dim>([&]<int I>() { at(Offset + I) = x[I]; });
    else
      at(Offset) = x;

    if constexpr (sizeof...(Xs)) constructFrom<Offset + dim>(xs...);
  }

  std::array<T, N> components{};
};

template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '+'>, N>>
R operator+(Vector<A, N> a, Vector<B, N> b) {
  return dispatchSequence<N>([&]<int... Is>() {
    return R((a[Is] + b[Is])...);
  });
}
template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '-'>, N>>
R operator-(Vector<A, N> a, Vector<B, N> b) {
  return dispatchSequence<N>([&]<int... Is>() {
    return R((a[Is] - b[Is])...);
  });
}
template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '*'>, N>>
R operator*(Vector<A, N> a, Vector<B, N> b) {
  return dispatchSequence<N>([&]<int... Is>() {
    return R((a[Is] * b[Is])...);
  });
}
template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '/'>, N>>
R operator/(Vector<A, N> a, Vector<B, N> b) {
  return dispatchSequence<N>([&]<int... Is>() {
    return R((a[Is] / b[Is])...);
  });
}
template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '+'>, N>>
R operator*(Vector<A, N> a, B b) {
  return dispatchSequence<N>([&]<int... Is>() { return R((a[Is] * b)...); });
}
template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '*'>, N>>
R operator*(A a, Vector<B, N> b) {
  return dispatchSequence<N>([&]<int... Is>() { return R((a * b[Is])...); });
}
template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '/'>, N>>
R operator/(Vector<A, N> a, B b) {
  return dispatchSequence<N>([&]<int... Is>() { return R((a[Is] / b)...); });
}
template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '/'>, N>>
R operator/(A a, Vector<B, N> b) {
  return dispatchSequence<N>([&]<int... Is>() { return R((a / b[Is])...); });
}

template <typename T, int N>
T innerProduct(Vector<T, N> a, Vector<T, N> b) {
  return dispatchSequence<N>([&]<int... Is>() {
    return sum((a[Is] * b[Is])...);
  });
}

template <typename T, int N>
T dot(Vector<T, N> a, Vector<T, N> b) {
  return innerProduct(a, b);
}

template <typename T, int N>
auto lengthSquared(Vector<T, N> x) {
  return dot(x, x);
}

template <typename T>
using Vector2 = Vector<T, 2>;
template <typename T>
using Vector3 = Vector<T, 3>;
template <typename T>
using Vector4 = Vector<T, 4>;

using vec2 = Vector2<float>;
using vec3 = Vector3<float>;
using vec4 = Vector4<float>;
using vec2i = Vector2<int>;
using vec3i = Vector3<int>;
using vec4i = Vector4<int>;
using vec2u8 = Vector2<uint8_t>;
using vec3u8 = Vector3<uint8_t>;
using vec4u8 = Vector4<uint8_t>;

template <typename T, int N, int M>
struct Matrix {
  static Matrix identity() {
    auto m = Matrix();
    for (int i = 0; i < std::min(N, M); ++i) m[i][i] = T(1);
    return m;
  }

  Matrix() = default;
  template <typename... Columns>
    requires(isVectorN<N, Columns> && ...)
  Matrix(Columns... columns) : columns(columns...) {}

  constexpr std::size_t size() const { return M; }

  Vector<T, M> row(int i) const {
    return dispatchSequence<M>([&]<int... Is>() {
      return Vector<T, M>(column(Is)[i]...);
    });
  }
  Vector<T, N>& column(int i) {
    DCHECK_GE(i, 0);
    DCHECK_LT(i, M);
    return columns[i];
  }
  const Vector<T, N>& column(int i) const {
    DCHECK_GE(i, 0);
    DCHECK_LT(i, M);
    return columns[i];
  }

  Vector<T, N>& operator[](int i) { return column(i); }
  const Vector<T, N>& operator[](int i) const { return column(i); }

  friend auto operator<=>(Matrix, Matrix) = default;

 private:
  Vector<Vector<T, N>, M> columns;
};

template <typename A, typename B, int N, int M,
          typename R = Vector<ArithmeticResult<A, B, '*'>, N>>
R operator*(Matrix<A, N, M> a, Vector<B, M> b) {
  return dispatchSequence<N>([&]<int... Is>() {
    return R(dot(a.row(Is), b)...);
  });
}
template <typename A, typename B, int N, int M,
          typename R = Matrix<ArithmeticResult<A, B, '*'>, N, M>>
R operator+(Matrix<A, N, M> a, Matrix<B, N, M> b) {
  return dispatchSequence<M>([&]<int... Is>() {
    return R((a[Is] + b[Is])...);
  });
}
template <typename A, typename B, int N, int M,
          typename R = Matrix<ArithmeticResult<A, B, '*'>, N, M>>
R operator-(Matrix<A, N, M> a, Matrix<B, N, M> b) {
  return dispatchSequence<M>([&]<int... Is>() {
    return R((a[Is] - b[Is])...);
  });
}
template <typename A, typename B, int N, int M, int K,
          typename R = Matrix<ArithmeticResult<A, B, '*'>, N, K>>
R operator*(Matrix<A, N, M> a, Matrix<B, M, K> b) {
  return dispatchSequence<K>([&]<int... Is>() { return R((a * b[Is])...); });
}

template <typename T, int N, int M>
auto lengthSquared(Matrix<T, N, M> x) {
  return dispatchSequence<M>([&]<int... Is>() {
    return sum(lengthSquared(x[Is])...);
  });
}

template <typename T>
using Matrix2 = Matrix<T, 2, 2>;
template <typename T>
using Matrix3 = Matrix<T, 3, 3>;
template <typename T>
using Matrix4 = Matrix<T, 4, 4>;

using mat2 = Matrix2<float>;
using mat3 = Matrix3<float>;
using mat4 = Matrix4<float>;

template <typename T, int N>
auto translate(Vector<T, N> x) {
  auto m = Matrix<T, N + 1, N + 1>::identity();
  m[N] = Vector<T, N + 1>(x, 1);
  return m;
}
template <typename T, int N>
auto scale(Vector<T, N> x) {
  auto m = Matrix<T, N, N>();
  for(int i = 0; i < N; i++)
  m[i][i] = x[i];
  return m;
}

}  // namespace pine