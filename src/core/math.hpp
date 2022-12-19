#pragma once
#include <core/string.hpp>
#include <core/check.hpp>

#include <cstdint>
#include <cmath>
#include <array>

namespace pine {

constexpr float Epsilon = std::numeric_limits<float>::epsilon();
constexpr float MaxFloat = std::numeric_limits<float>::max();
constexpr float Infinity = std::numeric_limits<float>::infinity();
constexpr float OneMinusEpsilon = 1.0f - Epsilon;
constexpr float E = 2.71828182846f;
constexpr float Pi = 3.1415926535f;

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

#define DCHECK_EQF(a, b) CHECK_EQF(a, b)

constexpr auto sum(auto... xs) { return (xs + ...); }

constexpr auto max(auto a, auto b) { return std::max(a, b); }
constexpr auto max(auto x, auto... xs) { return std::max(x, max(xs...)); }

constexpr auto min(auto a, auto b) { return std::min(a, b); }
constexpr auto min(auto x, auto... xs) { return std::min(x, min(xs...)); }

constexpr bool between(auto x, auto a, auto b) { return x >= a && x <= b; }

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

template <typename T>
constexpr int dimensionOf = 1;
template <typename T, int N>
constexpr int dimensionOf<Vector<T, N>> = N;

template <typename T>
constexpr bool isVector = false;
template <int N, typename T>
constexpr bool isVector<Vector<T, N>> = true;

template <int N, typename T>
constexpr bool isVectorN = false;
template <int N, typename T>
constexpr bool isVectorN<N, Vector<T, N>> = true;

template <typename T, int N>
struct Vector {
  static_assert(!isVector<T>, "Recursive Vector is not allowed");
  Vector() = default;

  template <typename X>
    requires(dimensionOf<X> == 1)
  explicit Vector(X x) {
    dispatchIndex<N>([&]<int I>() { at(I) = x; });
  }
  template <typename X>
    requires(dimensionOf<X> > N)
  Vector(X x) {
    dispatchIndex<N>([&]<int I>() { at(I) = x[I]; });
  }
  template <typename... Ts>
    requires(sum(dimensionOf<Ts>...) == N)
  explicit Vector(Ts... xs) {
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
  Vector& operator+=(Vector<B, N> b) {
    dispatchIndex<N>([&]<int I>() { at(I) += b[I]; });
    return *this;
  }
  template <typename B>
  Vector& operator-=(Vector<B, N> b) {
    dispatchIndex<N>([&]<int I>() { at(I) -= b[I]; });
    return *this;
  }
  template <typename B>
  Vector& operator*=(Vector<B, N> b) {
    dispatchIndex<N>([&]<int I>() { at(I) *= b[I]; });
    return *this;
  }
  template <typename B>
  Vector& operator/=(Vector<B, N> b) {
    dispatchIndex<N>([&]<int I>() { at(I) /= b[I]; });
    return *this;
  }
  template <typename B>
  Vector& operator*=(B b) {
    dispatchIndex<N>([&]<int I>() { at(I) *= b; });
    return *this;
  }
  template <typename B>
  Vector& operator/=(B b) {
    dispatchIndex<N>([&]<int I>() { at(I) /= b; });
    return *this;
  }

  Vector operator-() const {
    auto copy = *this;
    dispatchIndex<N>([&]<int I>() { copy[I] = -at(I); });
    return copy;
  }

  friend auto operator<=>(Vector, Vector) = default;

 private:
  template <int Offset, typename X, typename... Xs>
  void constructFrom(X x, Xs... xs) {
    constexpr int dim = dimensionOf<X>;

    if constexpr (dim > 1)
      dispatchIndex<dim>([&]<int I>() { at(Offset + I) = x[I]; });
    else
      at(Offset) = x;

    if constexpr (sizeof...(Xs)) constructFrom<Offset + dim>(xs...);
  }

  std::array<T, N> components{};
};

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

template <int I, typename T>
auto nthComponent(T x) {
  if constexpr (isVector<T>)
    return x[I];
  else
    return x;
}

template <int N, typename F, typename... Ts>
auto apply(F f, Ts... xs) {
  const auto applyAt = [&]<int I>() { return f(nthComponent<I>(xs)...); };

  return dispatchSequence<N>([&]<int... Is>() {
    using R = decltype(applyAt.template operator()<0>());
    return Vector<R, N>(applyAt.template operator()<Is>()...);
  });
}

template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '+'>, N>>
R operator+(Vector<A, N> a, Vector<B, N> b) {
  return apply<N>(std::plus(), a, b);
}
template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '-'>, N>>
R operator-(Vector<A, N> a, Vector<B, N> b) {
  return apply<N>(std::minus(), a, b);
}
template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '*'>, N>>
R operator*(Vector<A, N> a, Vector<B, N> b) {
  return apply<N>(std::multiplies(), a, b);
}
template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '/'>, N>>
R operator/(Vector<A, N> a, Vector<B, N> b) {
  return apply<N>(std::divides(), a, b);
}
template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '+'>, N>>
R operator*(Vector<A, N> a, B b) {
  return apply<N>(std::multiplies(), a, b);
}
template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '*'>, N>>
R operator*(A a, Vector<B, N> b) {
  return apply<N>(std::multiplies(), a, b);
}
template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '/'>, N>>
R operator/(Vector<A, N> a, B b) {
  return apply<N>(std::divides(), a, b);
}
template <typename A, typename B, int N,
          typename R = Vector<ArithmeticResult<A, B, '/'>, N>>
R operator/(A a, Vector<B, N> b) {
  return apply<N>(std::divides(), a, b);
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

template <typename T>
Vector3<T> cross(Vector3<T> a, Vector3<T> b) {
  return Vector3<T>(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0]);
}

template <typename T, int N>
T lengthSquared(Vector<T, N> x) {
  return dot(x, x);
}

template <typename T, int N>
Vector<T, N> normalize(Vector<T, N> x) {
  return x / length(x);
}

template <typename T, typename IndexType, int M, int N>
Vector<T, N> shuffle(Vector<T, M> x, Vector<IndexType, N> index) {
  return apply<N>([&](int i) { return x[i]; }, index);
}

template <typename T, int N, int I>
Vector<T, N> oneAt() {
  auto x = Vector<T, N>();
  x[I] = T(1);
  return x;
}

template <typename T, int N>
Vector<T, N> abs(Vector<T, N> x) {
  return apply<N>([](auto x) { return std::abs(x); }, x);
}

template <typename T, int N>
Vector<T, N> sqrt(Vector<T, N> x) {
  return apply<N>([](auto x) { return std::sqrt(x); }, x);
}

template <typename T, int N>
Vector<T, N> exp(Vector<T, N> x) {
  return apply<N>([](auto x) { return std::exp(x); }, x);
}

template <typename T, int N>
Vector<T, N> log(Vector<T, N> x) {
  return apply<N>([](auto x) { return std::log(x); }, x);
}

template <typename T, int N>
Vector<T, N> pow(Vector<T, N> base, auto exp) {
  return apply<N>([](auto base, auto exp) { return std::pow(base, exp); }, base,
                  exp);
}

template <typename T>
T clamp(T x, T a, T b) {
  return std::max(std::min(x, b), a);
}
template <typename T, int N>
Vector<T, N> clamp(Vector<T, N> x, Vector<T, N> a, Vector<T, N> b) {
  return apply<N>(clamp<T>, x, a, b);
}

template <typename X, typename T>
X lerp(X a, X b, T t) {
  return a * (T(1) - t) + b * t;
}
template <typename X, typename T, int N>
Vector<T, N> lerp(Vector<X, N> a, Vector<X, N> b, T t) {
  return apply<N>(lerp<X, T>, a, b, t);
}

template <typename T, int N, int M>
struct Matrix {
  static Matrix identity() {
    auto m = Matrix();
    for (int i = 0; i < std::min(N, M); ++i) m[i][i] = T(1);
    return m;
  }

  Matrix() = default;
  template <typename... Columns>
    requires(sizeof...(Columns) == M && (isVectorN<N, Columns> && ...))
  explicit Matrix(Columns... columns) : columns{columns...} {}

  template <typename B, int N1, int M1>
  explicit Matrix(Matrix<B, N1, M1> b) {
    for (int c = 0; c < std::min(N, N1); ++c)
      for (int r = 0; r < std::min(M, M1); ++r) column(c)[r] = b[c][r];
  }

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
  std::array<Vector<T, N>, M> columns;
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
Matrix<T, M, N> transpose(Matrix<T, N, M> x) {
  return dispatchSequence<N>([&]<int... Is>() {
    return Matrix<T, M, N>(x.row(Is)...);
  });
}

template <typename T, int N, int M>
T lengthSquared(Matrix<T, N, M> x) {
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

template <typename T>
T determinant(Matrix2<T> x) {
  return x[0][0] * x[1][1] - x[0][1] * x[1][0];
}
template <typename T>
T determinant(Matrix3<T> x) {
  return dot(x[0], cross(x[1], x[2]));
}
template <typename T>
T determinant(Matrix4<T> x) {
  auto d = T(0);

  const auto subMatrixDeterminant = [&](int i) {
    const auto indices = vec3i((i + 1) % 4, (i + 2) % 4, (i + 3) % 4);
    return dot(shuffle(x[1], indices),
               cross(shuffle(x[2], indices), shuffle(x[3], indices)));
  };
  for (int i = 0; i < 4; ++i) d = -d + x[0][i] * subMatrixDeterminant(i);
  return -d;
}

template <int column, typename T, int N, int M>
Matrix<T, N, M> replaceColumn(Matrix<T, N, M> m, Vector<T, N> x) {
  m[column] = x;
  return m;
}

template <typename T, int N>
Vector<T, N> solve(Matrix<T, N, N> m, Vector<T, N> b) {
  const auto d = determinant(m);
  return dispatchSequence<N>([&]<int... Is>() {
    return Vector<T, N>((determinant(replaceColumn<Is>(m, b)) / d)...);
  });
}

template <typename T, int N>
Matrix<T, N, N> inverse(Matrix<T, N, N> m) {
  return dispatchSequence<N>([&]<int... Is>() {
    return Matrix<T, N, N>(solve(m, oneAt<T, N, Is>())...);
  });
}

inline mat3 coordinateSystem(vec3 n) {
  DCHECK_EQ(lengthSquared(n), 1);
  auto m = mat3();
  m[2] = n;

  if (std::abs(n[0]) < 0.5f)
    m[0] = normalize(cross(n, vec3(1, 0, 0)));
  else
    m[0] = normalize(cross(n, vec3(0, 1, 0)));

  m[1] = cross(m[0], m[1]);

  return m;
}

inline mat4 lookAt(vec3 from, vec3 to, vec3 up = vec3(0, 1, 0)) {
  auto m = mat4();
  m[2] = vec4(normalize(to - from), 0);
  m[0] = vec4(normalize(cross<float>(up, m[2])), 0);
  m[1] = vec4(cross<float>(m[0], m[2]), 0);
  m[3] = vec4(from, 1);
  return m;
}

template <typename T, int N>
Matrix<T, N + 1, N + 1> translate(Vector<T, N> x) {
  auto m = Matrix<T, N + 1, N + 1>::identity();
  m[N] = Vector<T, N + 1>(x, 1);
  return m;
}
template <typename T, int N>
Matrix<T, N, N> scale(Vector<T, N> x) {
  auto m = Matrix<T, N, N>();
  for (int i = 0; i < N; i++) m[i][i] = x[i];
  return m;
}

}  // namespace pine