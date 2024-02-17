#pragma once

#include <pine/core/math.h>

#include <psl/string.h>

namespace pine {

template <typename T>
struct Vector2;
template <typename T>
struct Vector3;
template <typename T>
struct Vector4;

template <typename T>
struct Matrix2;
template <typename T>
struct Matrix3;
template <typename T>
struct Matrix4;

template <typename T>
constexpr bool is_pine_vector = false;
template <typename T>
constexpr bool is_pine_vector<Vector2<T>> = true;
template <typename T>
constexpr bool is_pine_vector<Vector3<T>> = true;
template <typename T>
constexpr bool is_pine_vector<Vector4<T>> = true;
template <typename T>
constexpr bool is_pine_matrix = false;
template <typename T>
constexpr bool is_pine_matrix<Matrix2<T>> = true;
template <typename T>
constexpr bool is_pine_matrix<Matrix3<T>> = true;
template <typename T>
constexpr bool is_pine_matrix<Matrix4<T>> = true;
template <typename T>
constexpr bool is_pine_vector_or_matrix = is_pine_vector<T> || is_pine_matrix<T>;

template <typename T>
struct Vector2 {
  Vector2() = default;
  explicit Vector2(auto v) : x(T(v)), y(T(v)) {
  }
  Vector2(auto x, auto y) : x(T(x)), y(T(y)) {
  }
  template <typename U>
  Vector2(const Vector2<U> &v) : x(T(v.x)), y(T(v.y)) {
  }
  template <typename U>
  explicit Vector2(Vector3<U> v) : x(T(v.x)), y(T(v.y)) {
  }
  template <typename U>
  explicit Vector2(Vector4<U> v) : x(T(v.x)), y(T(v.y)) {
  }

  template <typename U>
  Vector2 &operator+=(Vector2<U> rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
  }
  template <typename U>
  Vector2 &operator-=(Vector2<U> rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
  }
  template <typename U>
  Vector2 &operator*=(Vector2<U> rhs) {
    x *= rhs.x;
    y *= rhs.y;
    return *this;
  }
  template <typename U>
  Vector2 &operator/=(Vector2<U> rhs) {
    x /= rhs.x;
    y /= rhs.y;
    return *this;
  }
  template <psl::Integral U>
  requires psl::Integral<T>
  Vector2 &operator%=(Vector2<U> rhs) {
    x %= rhs.x;
    y %= rhs.y;
    return *this;
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  Vector2 &operator*=(U rhs) {
    x *= rhs;
    y *= rhs;
    return *this;
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  Vector2 &operator/=(U rhs) {
    x /= rhs;
    y /= rhs;
    return *this;
  }

  template <typename U>
  friend Vector2<psl::OpResult<T, U, '+'>> operator+(Vector2<T> lhs, Vector2<U> rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y};
  }
  template <typename U>
  friend Vector2<psl::OpResult<T, U, '-'>> operator-(Vector2<T> lhs, Vector2<U> rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y};
  }
  template <typename U>
  friend Vector2<psl::OpResult<T, U, '*'>> operator*(Vector2<T> lhs, Vector2<U> rhs) {
    return {lhs.x * rhs.x, lhs.y * rhs.y};
  }
  template <typename U>
  friend Vector2<psl::OpResult<T, U, '/'>> operator/(Vector2<T> lhs, Vector2<U> rhs) {
    return {lhs.x / rhs.x, lhs.y / rhs.y};
  }
  template <psl::Integral U>
  requires psl::Integral<T>
  friend Vector2<psl::OpResult<T, U, '%'>> operator%(Vector2<T> lhs, Vector2<U> rhs) {
    return {lhs.x % rhs.x, lhs.y % rhs.y};
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  friend Vector2<psl::OpResult<T, U, '*'>> operator*(Vector2<T> lhs, U rhs) {
    return {lhs.x * rhs, lhs.y * rhs};
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  friend Vector2<psl::OpResult<T, U, '/'>> operator/(Vector2<T> lhs, U rhs) {
    return {lhs.x / rhs, lhs.y / rhs};
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  friend Vector2<psl::OpResult<U, T, '*'>> operator*(U lhs, Vector2<T> rhs) {
    return {lhs * rhs.x, lhs * rhs.y};
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  friend Vector2<psl::OpResult<U, T, '/'>> operator/(U lhs, Vector2<T> rhs) {
    return {lhs / rhs.x, lhs / rhs.y};
  }
  template <typename U>
  friend bool operator==(Vector2<T> lhs, Vector2<U> rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
  }
  template <typename U>
  friend bool operator!=(Vector2<T> lhs, Vector2<U> rhs) {
    return !(lhs == rhs);
  }
  bool has_nan() const {
    return psl::isnan(x) || psl::isnan(y);
  }
  bool has_inf() const {
    return psl::isinf(x) || psl::isinf(y);
  }
  bool is_black() const {
    return *this == Vector2{};
  }

  Vector2 operator-() const {
    return {-x, -y};
  }

  T &operator[](int i) {
    return (&x)[i];
  }
  const T &operator[](int i) const {
    return (&x)[i];
  }

  T x{0}, y{0};
};

template <typename T>
struct Vector3 {
  Vector3() = default;
  explicit Vector3(auto v) : x(T(v)), y(T(v)), z(T(v)) {
  }
  Vector3(auto x, auto y, auto z) : x(T(x)), y(T(y)), z(T(z)) {
  }
  template <typename U>
  explicit Vector3(Vector2<U> xy) : x(T(xy.x)), y(T(xy.y)), z(T(0)) {
  }
  template <typename U>
  Vector3(const Vector3<U> &v) : x(T(v.x)), y(T(v.y)), z(T(v.z)) {
  }
  template <typename U>
  explicit Vector3(Vector4<U> v) : x(T(v.x)), y(T(v.y)), z(T(v.z)) {
  }
  template <typename U>
  Vector3(Vector2<U> xy, U z) : x(T(xy.x)), y(T(xy.y)), z(T(z)) {
  }
  template <typename U>
  Vector3(U x, Vector2<U> yz) : x(T(x)), y(T(yz.x)), z(T(yz.y)) {
  }

  template <typename U>
  Vector3 &operator+=(Vector3<U> rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }
  template <typename U>
  Vector3 &operator-=(Vector3<U> rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }
  template <typename U>
  Vector3 &operator*=(Vector3<U> rhs) {
    x *= rhs.x;
    y *= rhs.y;
    z *= rhs.z;
    return *this;
  }
  template <typename U>
  Vector3 &operator/=(Vector3<U> rhs) {
    x /= rhs.x;
    y /= rhs.y;
    z /= rhs.z;
    return *this;
  }
  template <psl::Integral U>
  requires psl::Integral<T>
  Vector3 &operator%=(Vector3<U> rhs) {
    x %= rhs.x;
    y %= rhs.y;
    z %= rhs.z;
    return *this;
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  Vector3 &operator*=(U rhs) {
    x *= rhs;
    y *= rhs;
    z *= rhs;
    return *this;
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  Vector3 &operator/=(U rhs) {
    x /= rhs;
    y /= rhs;
    z /= rhs;
    return *this;
  }

  template <typename U>
  friend Vector3<psl::OpResult<T, U, '+'>> operator+(Vector3<T> lhs, Vector3<U> rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
  }
  template <typename U>
  friend Vector3<psl::OpResult<T, U, '-'>> operator-(Vector3<T> lhs, Vector3<U> rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
  }
  template <typename U>
  friend Vector3<psl::OpResult<T, U, '*'>> operator*(Vector3<T> lhs, Vector3<U> rhs) {
    return {lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z};
  }
  template <typename U>
  friend Vector3<psl::OpResult<T, U, '/'>> operator/(Vector3<T> lhs, Vector3<U> rhs) {
    return {lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z};
  }
  template <psl::Integral U>
  requires psl::Integral<T>
  friend Vector3<psl::OpResult<T, U, '%'>> operator%(Vector3<T> lhs, Vector3<U> rhs) {
    return {lhs.x % rhs.x, lhs.y % rhs.y, lhs.z % rhs.z};
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  friend Vector3<psl::OpResult<T, U, '*'>> operator*(Vector3<T> lhs, U rhs) {
    return {lhs.x * rhs, lhs.y * rhs, lhs.z * rhs};
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  friend Vector3<psl::OpResult<T, U, '/'>> operator/(Vector3<T> lhs, U rhs) {
    return {lhs.x / rhs, lhs.y / rhs, lhs.z / rhs};
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  friend Vector3<psl::OpResult<U, T, '*'>> operator*(U lhs, Vector3<T> rhs) {
    return {lhs * rhs.x, lhs * rhs.y, lhs * rhs.z};
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  friend Vector3<psl::OpResult<U, T, '/'>> operator/(U lhs, Vector3<T> rhs) {
    return {lhs / rhs.x, lhs / rhs.y, lhs / rhs.z};
  }
  template <typename U>
  friend bool operator==(Vector3<T> lhs, Vector3<U> rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
  }
  template <typename U>
  friend bool operator!=(Vector3<T> lhs, Vector3<U> rhs) {
    return lhs.x != rhs.x || lhs.y != rhs.y || lhs.z != rhs.z;
  }

  bool has_nan() const {
    return psl::isnan(x) || psl::isnan(y) || psl::isnan(z);
  }
  bool has_inf() const {
    return psl::isinf(x) || psl::isinf(y) || psl::isinf(z);
  }
  bool is_black() const {
    return *this == Vector3{};
  }

  Vector3 operator-() const {
    return {-x, -y, -z};
  }

  T &operator[](int i) {
    return (&x)[i];
  }
  const T &operator[](int i) const {
    return (&x)[i];
  }

  T x{0}, y{0}, z{0};
};

template <typename T>
struct Vector4 {
  Vector4() = default;
  explicit Vector4(auto v) : x(T(v)), y(T(v)), z(T(v)), w(T(v)) {
  }
  Vector4(auto x, auto y, auto z, auto w) : x(T(x)), y(T(y)), z(T(z)), w(T(w)) {
  }
  template <typename U>
  explicit Vector4(Vector2<U> v) : x(T(v.x)), y(T(v.y)), z(T(0)), w(T(0)) {
  }
  template <typename U>
  explicit Vector4(Vector3<U> v) : x(T(v.x)), y(T(v.y)), z(T(v.z)), w(T(0)) {
  }
  template <typename U>
  Vector4(const Vector4<U> &v) : x(T(v.x)), y(T(v.y)), z(T(v.z)), w(T(v.w)) {
  }
  template <typename U>
  Vector4(Vector2<U> xy, Vector2<U> zw) : x(T(xy.x)), y(T(xy.y)), z(T(zw.x)), w(T(zw.y)) {
  }
  template <typename U>
  Vector4(Vector2<U> xy, U z, U w) : x(T(xy.x)), y(T(xy.y)), z(T(z)), w(T(w)) {
  }
  template <typename U>
  Vector4(Vector3<U> xyz, U w) : x(T(xyz.x)), y(T(xyz.y)), z(T(xyz.z)), w(T(w)) {
  }
  template <typename U>
  Vector4(U x, Vector3<U> yzw) : x(T(x)), y(T(yzw.x)), z(T(yzw.y)), w(T(yzw.z)) {
  }

  template <typename U>
  Vector4 &operator+=(Vector4<U> rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    w += rhs.w;
    return *this;
  }
  template <typename U>
  Vector4 &operator-=(Vector4<U> rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    w -= rhs.w;
    return *this;
  }
  template <typename U>
  Vector4 &operator*=(Vector4<U> rhs) {
    x *= rhs.x;
    y *= rhs.y;
    z *= rhs.z;
    w *= rhs.w;
    return *this;
  }
  template <typename U>
  Vector4 &operator/=(Vector4<U> rhs) {
    x /= rhs.x;
    y /= rhs.y;
    z /= rhs.z;
    w /= rhs.w;
    return *this;
  }
  template <psl::Integral U>
  requires psl::Integral<T>
  Vector4 &operator%=(Vector4<U> rhs) {
    x %= rhs.x;
    y %= rhs.y;
    z %= rhs.z;
    w %= rhs.w;
    return *this;
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  Vector4 &operator*=(U rhs) {
    x *= rhs;
    y *= rhs;
    z *= rhs;
    w *= rhs;
    return *this;
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  Vector4 &operator/=(U rhs) {
    x /= rhs;
    y /= rhs;
    z /= rhs;
    w /= rhs;
    return *this;
  }

  template <typename U>
  friend Vector4<psl::OpResult<T, U, '+'>> operator+(Vector4<T> lhs, Vector4<U> rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w};
  }
  template <typename U>
  friend Vector4<psl::OpResult<T, U, '-'>> operator-(Vector4<T> lhs, Vector4<U> rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w};
  }
  template <typename U>
  friend Vector4<psl::OpResult<T, U, '*'>> operator*(Vector4<T> lhs, Vector4<U> rhs) {
    return {lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z, lhs.w * rhs.w};
  }
  template <typename U>
  friend Vector4<psl::OpResult<T, U, '/'>> operator/(Vector4<T> lhs, Vector4<U> rhs) {
    return {lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z, lhs.w / rhs.w};
  }
  template <psl::Integral U>
  requires psl::Integral<T>
  friend Vector4<psl::OpResult<T, U, '%'>> operator%(Vector4<T> lhs, Vector4<U> rhs) {
    return {lhs.x % rhs.x, lhs.y % rhs.y, lhs.z % rhs.z, lhs.w % rhs.w};
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  friend Vector4<psl::OpResult<T, U, '*'>> operator*(Vector4<T> lhs, U rhs) {
    return {lhs.x * rhs, lhs.y * rhs, lhs.z * rhs, lhs.w * rhs};
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  friend Vector4<psl::OpResult<T, U, '/'>> operator/(Vector4<T> lhs, U rhs) {
    return {lhs.x / rhs, lhs.y / rhs, lhs.z / rhs, lhs.w / rhs};
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  friend Vector4<psl::OpResult<U, T, '*'>> operator*(U lhs, Vector4<T> rhs) {
    return {lhs * rhs.x, lhs * rhs.y, lhs * rhs.z, lhs * rhs.w};
  }
  template <typename U>
  requires(!is_pine_vector_or_matrix<U>)
  friend Vector4<psl::OpResult<U, T, '/'>> operator/(U lhs, Vector4<T> rhs) {
    return {lhs / rhs.x, lhs / rhs.y, lhs / rhs.z, lhs / rhs.w};
  }
  template <typename U>
  friend bool operator==(Vector4<T> lhs, Vector4<U> rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w;
  }
  template <typename U>
  friend bool operator!=(Vector4<T> lhs, Vector4<U> rhs) {
    return lhs.x != rhs.x || lhs.y != rhs.y || lhs.z != rhs.z || lhs.w != rhs.w;
  }

  bool has_nan() const {
    return psl::isnan(x) || psl::isnan(y) || psl::isnan(z) || psl::isnan(w);
  }
  bool has_inf() const {
    return psl::isinf(x) || psl::isinf(y) || psl::isinf(z) || psl::isinf(w);
  }
  bool is_black() const {
    return *this == Vector4{};
  }

  Vector4 operator-() const {
    return {-x, -y, -z, -w};
  }

  T &operator[](int i) {
    return (&x)[i];
  }
  const T &operator[](int i) const {
    return (&x)[i];
  }

  T x{0}, y{0}, z{0}, w{0};
};

template <typename T>
struct Matrix2 {
  static Matrix2 zeros() {
    return Matrix2(0, 0, 0, 0);
  }

  static Matrix2 identity() {
    return Matrix2(1, 0, 1, 0);
  }

  Matrix2() {
    *this = identity();
  }

  Matrix2(T x0, T y0, T x1, T y1) : x(x0, x1), y(y0, y1) {
  }

  Matrix2(Vector2<T> x, Vector2<T> y) : x(x), y(y){};

  Matrix2 &operator+=(const Matrix2 &rhs) {
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++)
        (*this)[c][r] += rhs[c][r];
    return *this;
  }

  Matrix2 &operator-=(const Matrix2 &rhs) {
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++)
        (*this)[c][r] -= rhs[c][r];
    return *this;
  }

  Matrix2 &operator*=(T rhs) {
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++)
        (*this)[c][r] *= rhs;
    return *this;
  }

  Matrix2 &operator/=(T rhs) {
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++)
        (*this)[c][r] /= rhs;
    return *this;
  }

  friend Matrix2 operator*(Matrix2 lhs, T rhs) {
    return lhs *= rhs;
  }
  friend Matrix2 operator/(Matrix2 lhs, T rhs) {
    return lhs /= rhs;
  }

  friend Matrix2 operator*(const Matrix2 &lhs, const Matrix2 &rhs) {
    Matrix2 ret = zeros();
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++) {
        for (int i = 0; i < N; i++)
          ret[c][r] += lhs[i][r] * rhs[c][i];
      }

    return ret;
  }

  Vector2<T> row(int i) const {
    return {x[i], y[i]};
  }

  Vector2<T> &operator[](int i) {
    return (&x)[i];
  }
  const Vector2<T> &operator[](int i) const {
    return (&x)[i];
  }

  friend bool operator==(const Matrix2 &m0, const Matrix2 &m1) {
    return m0.x == m1.x && m0.y == m1.y;
  }
  friend bool operator!=(const Matrix2 &m0, const Matrix2 &m1) {
    return m0.x != m1.x || m0.y != m1.y;
  }

  static constexpr int N = 2;
  Vector2<T> x, y;
};

template <typename T>
struct Matrix3 {
  static Matrix3 zeros() {
    return Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  static Matrix3 identity() {
    return Matrix3(1, 0, 0, 0, 1, 0, 0, 0, 1);
  }

  Matrix3() {
    *this = identity();
  }

  Matrix3(T x0, T y0, T z0, T x1, T y1, T z1, T x2, T y2, T z2)
      : x(x0, x1, x2), y(y0, y1, y2), z(z0, z1, z2) {
  }

  Matrix3(Vector3<T> x, Vector3<T> y, Vector3<T> z) : x(x), y(y), z(z){};

  Matrix3 &operator+=(const Matrix3 &rhs) {
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++)
        (*this)[c][r] += rhs[c][r];
    return *this;
  }

  Matrix3 &operator-=(const Matrix3 &rhs) {
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++)
        (*this)[c][r] -= rhs[c][r];
    return *this;
  }

  Matrix3 &operator*=(T rhs) {
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++)
        (*this)[c][r] *= rhs;
    return *this;
  }

  Matrix3 &operator/=(T rhs) {
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++)
        (*this)[c][r] /= rhs;
    return *this;
  }

  friend Matrix3 operator*(Matrix3 lhs, T rhs) {
    return lhs *= rhs;
  }
  friend Matrix3 operator/(Matrix3 lhs, T rhs) {
    return lhs /= rhs;
  }

  friend Matrix3 operator*(const Matrix3 &lhs, const Matrix3 &rhs) {
    Matrix3 ret = zeros();
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++) {
        for (int i = 0; i < N; i++)
          ret[c][r] += lhs[i][r] * rhs[c][i];
      }

    return ret;
  }

  Vector3<T> row(int i) const {
    return {x[i], y[i], z[i]};
  }

  Vector3<T> &operator[](int i) {
    return (&x)[i];
  }
  const Vector3<T> &operator[](int i) const {
    return (&x)[i];
  }

  friend bool operator==(const Matrix3 &m0, const Matrix3 &m1) {
    return m0.x == m1.x && m0.y == m1.y && m0.z == m1.z;
  }
  friend bool operator!=(const Matrix3 &m0, const Matrix3 &m1) {
    return m0.x != m1.x || m0.y != m1.y || m0.z != m1.z;
  }

  static constexpr int N = 3;
  Vector3<T> x, y, z;
};

template <typename T>
struct Matrix4 {
  static Matrix4 zeros() {
    return Matrix4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  static Matrix4 identity() {
    return Matrix4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
  }

  Matrix4() {
    *this = identity();
  }

  Matrix4(T x0, T y0, T z0, T w0, T x1, T y1, T z1, T w1, T x2, T y2, T z2, T w2, T x3, T y3, T z3,
          T w3)
      : x(x0, x1, x2, x3), y(y0, y1, y2, y3), z(z0, z1, z2, z3), w(w0, w1, w2, w3) {
  }

  Matrix4(Vector4<T> x, Vector4<T> y, Vector4<T> z, Vector4<T> w) : x(x), y(y), z(z), w(w){};

  operator Matrix3<T>() const {
    return Matrix3<T>(Vector3<T>{x}, Vector3<T>{y}, Vector3<T>{z});
  }

  Matrix4 &operator+=(const Matrix4 &rhs) {
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++)
        (*this)[c][r] += rhs[c][r];
    return *this;
  }

  Matrix4 &operator-=(const Matrix4 &rhs) {
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++)
        (*this)[c][r] -= rhs[c][r];
    return *this;
  }

  Matrix4 &operator*=(T rhs) {
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++)
        (*this)[c][r] *= rhs;
    return *this;
  }

  Matrix4 &operator/=(T rhs) {
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++)
        (*this)[c][r] /= rhs;
    return *this;
  }

  friend Matrix4 operator*(Matrix4 lhs, T rhs) {
    return lhs *= rhs;
  }
  friend Matrix4 operator/(Matrix4 lhs, T rhs) {
    return lhs /= rhs;
  }

  friend Matrix4 operator*(const Matrix4 &lhs, const Matrix4 &rhs) {
    Matrix4 ret = zeros();
    for (int c = 0; c < N; c++)
      for (int r = 0; r < N; r++) {
        for (int i = 0; i < N; i++)
          ret[c][r] += lhs[i][r] * rhs[c][i];
      }
    return ret;
  }

  Vector4<T> row(int i) const {
    return {x[i], y[i], z[i], w[i]};
  }

  Vector4<T> &operator[](int i) {
    return (&x)[i];
  }
  const Vector4<T> &operator[](int i) const {
    return (&x)[i];
  }

  friend bool operator==(const Matrix4 &m0, const Matrix4 &m1) {
    return m0.x == m1.x && m0.y == m1.y && m0.z == m1.z && m0.w == m1.w;
  }
  friend bool operator!=(const Matrix4 &m0, const Matrix4 &m1) {
    return m0.x != m1.x || m0.y != m1.y || m0.z != m1.z || m0.w != m1.w;
  }

  static constexpr int N = 4;
  Vector4<T> x, y, z, w;
};

typedef Vector2<uint8_t> vec2u8;
typedef Vector3<uint8_t> vec3u8;
typedef Vector4<uint8_t> vec4u8;
typedef Vector2<uint32_t> vec2u32;
typedef Vector3<uint32_t> vec3u32;
typedef Vector4<uint32_t> vec4u32;
typedef Vector2<int> vec2i;
typedef Vector3<int> vec3i;
typedef Vector4<int> vec4i;
typedef Vector2<int64_t> vec2i64;
typedef Vector3<int64_t> vec3i64;
typedef Vector4<int64_t> vec4i64;
typedef Vector2<float> vec2;
typedef Vector2<double> vec2d;
typedef Vector3<float> vec3;
typedef Vector4<float> vec4;
typedef Matrix2<float> mat2;
typedef Matrix3<float> mat3;
typedef Matrix4<float> mat4;

template <typename T>
psl::string to_string(Vector2<T> x) {
  return psl::to_string('[', x[0], ' ', x[1], ']');
}
template <typename T>
psl::string to_string(Vector3<T> x) {
  return psl::to_string('[', x[0], ' ', x[1], ' ', x[2], ']');
}
template <typename T>
psl::string to_string(Vector4<T> x) {
  return psl::to_string('[', x[0], ' ', x[1], ' ', x[2], ' ', x[3], ']');
}
template <typename T>
psl::string to_string(Matrix2<T> x) {
  return psl::to_string('[', x[0][1], ' ', x[1][1], "; ", x[0][0], ' ', x[1][0], ']');
}
template <typename T>
psl::string to_string(Matrix3<T> x) {
  return psl::to_string('[', x[0][2], ' ', x[1][2], ' ', x[2][2], "; ", x[0][1], ' ', x[1][1], ' ',
                        x[2][1], "; ", x[0][0], ' ', x[1][0], ' ', x[2][0], ']');
}
template <typename T>
psl::string to_string(Matrix4<T> x) {
  return psl::to_string('[', x[0][3], ' ', x[1][3], ' ', x[2][3], ' ', x[3][3], "; ", x[0][2], ' ',
                        x[1][2], ' ', x[2][2], ' ', x[3][2], "; ", x[0][1], ' ', x[1][1], ' ',
                        x[2][1], ' ', x[3][1], "; ", x[0][0], ' ', x[1][0], ' ', x[2][0], ' ',
                        x[3][0], ']');
}

template <typename T>
inline Vector2<T> operator*(const Matrix2<T> &m, const Vector2<T> &v) {
  return m.x * v.x + m.y * v.y;
}

template <typename T>
Vector3<T> operator*(const Matrix3<T> &m, const Vector3<T> &v) {
  return m.x * v.x + m.y * v.y + m.z * v.z;
}

template <typename T>
Vector4<T> operator*(const Matrix4<T> &m, const Vector4<T> &v) {
  return m.x * v.x + m.y * v.y + m.z * v.z + m.w * v.w;
}

template <typename T>
T length_squared(Vector2<T> v) {
  return v.x * v.x + v.y * v.y;
}
template <typename T>
T length_squared(Vector3<T> v) {
  return v.x * v.x + v.y * v.y + v.z * v.z;
}
template <typename T>
T length_squared(Vector4<T> v) {
  return v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w;
}

template <typename T>
auto length(T v) {
  using psl::sqrt;
  return sqrt(length_squared(v));
}

template <typename T>
auto distance_squared(T lhs, T rhs) {
  return length_squared(lhs - rhs);
}
template <typename T>
auto distance(T lhs, T rhs) {
  return length(lhs - rhs);
}

template <typename T>
auto normalize(T v) {
  auto len = length(v);
  if (len == 0)
    return v;
  return v / len;
}
template <typename T, typename U>
auto normalize(T v, U &len) {
  len = length(v);
  if (len == 0)
    return v;
  return v / len;
}

template <typename T>
T dot(Vector2<T> lhs, Vector2<T> rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}
template <typename T>
T dot(Vector3<T> lhs, Vector3<T> rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}
template <typename T>
T dot(Vector4<T> lhs, Vector4<T> rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
}
template <typename T>
auto absdot(T lhs, T rhs) {
  return psl::abs(dot(lhs, rhs));
}

template <typename T>
Vector3<T> cross(Vector3<T> lhs, Vector3<T> rhs) {
  return {lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z,
          lhs.x * rhs.y - lhs.y * rhs.x};
}

template <typename T>
T area(Vector2<T> v) {
  return v.x * v.y;
}

template <typename T>
T volume(Vector3<T> v) {
  return v.x * v.y * v.z;
}

template <typename T>
T sum(Vector2<T> v) {
  return v[0] + v[1];
}
template <typename T>
T sum(Vector3<T> v) {
  return v[0] + v[1] + v[2];
}
template <typename T>
T sum(Vector4<T> v) {
  return v[0] + v[1] + v[2] + v[3];
}

template <typename T>
Vector2<T> min(Vector2<T> lhs, Vector2<T> rhs) {
  return {psl::min(lhs.x, rhs.x), psl::min(lhs.y, rhs.y)};
}
template <typename T>
Vector3<T> min(Vector3<T> lhs, Vector3<T> rhs) {
  return {psl::min(lhs.x, rhs.x), psl::min(lhs.y, rhs.y), psl::min(lhs.z, rhs.z)};
}
template <typename T>
Vector4<T> min(Vector4<T> lhs, Vector4<T> rhs) {
  return {psl::min(lhs.x, rhs.x), psl::min(lhs.y, rhs.y), psl::min(lhs.z, rhs.z),
          psl::min(lhs.w, rhs.w)};
}

template <typename T>
Vector2<T> max(Vector2<T> lhs, Vector2<T> rhs) {
  return {psl::max(lhs.x, rhs.x), psl::max(lhs.y, rhs.y)};
}
template <typename T>
Vector3<T> max(Vector3<T> lhs, Vector3<T> rhs) {
  return {psl::max(lhs.x, rhs.x), psl::max(lhs.y, rhs.y), psl::max(lhs.z, rhs.z)};
}
template <typename T>
Vector4<T> max(Vector4<T> lhs, Vector4<T> rhs) {
  return {psl::max(lhs.x, rhs.x), psl::max(lhs.y, rhs.y), psl::max(lhs.z, rhs.z),
          psl::max(lhs.w, rhs.w)};
}

template <typename T>
Vector2<T> clamp(Vector2<T> val, Vector2<T> min, Vector2<T> max) {
  return {psl::clamp(val.x, min.x, max.x), psl::clamp(val.y, min.y, max.y)};
}
template <typename T>
Vector3<T> clamp(Vector3<T> val, Vector3<T> min, Vector3<T> max) {
  return {psl::clamp(val.x, min.x, max.x), psl::clamp(val.y, min.y, max.y),
          psl::clamp(val.z, min.z, max.z)};
}
template <typename T>
Vector4<T> clamp(Vector4<T> val, Vector4<T> min, Vector4<T> max) {
  return {psl::clamp(val.x, min.x, max.x), psl::clamp(val.y, min.y, max.y),
          psl::clamp(val.z, min.z, max.z), psl::clamp(val.w, min.w, max.w)};
}

template <typename T>
Vector2<T> lerp(Vector2<T> val, Vector2<T> min, Vector2<T> max) {
  return {psl::lerp(val.x, min.x, max.x), psl::lerp(val.y, min.y, max.y)};
}
template <typename T>
Vector3<T> lerp(Vector3<T> val, Vector3<T> min, Vector3<T> max) {
  return {psl::lerp(val.x, min.x, max.x), psl::lerp(val.y, min.y, max.y),
          psl::lerp(val.z, min.z, max.z)};
}
template <typename T>
Vector4<T> lerp(Vector4<T> val, Vector4<T> min, Vector4<T> max) {
  return {psl::lerp(val.x, min.x, max.x), psl::lerp(val.y, min.y, max.y),
          psl::lerp(val.z, min.z, max.z), psl::lerp(val.w, min.w, max.w)};
}
template <typename T, typename U>
auto lerp(T val, U min, U max) {
  return val * max + (T{1} - val) * min;
}
template <typename T, typename U>
auto lerp(T u, T v, U a, U b, U c) {
  return (T{1} - u - v) * a + u * b + v * c;
}

template <typename T>
Vector2<T> fract(Vector2<T> val) {
  return {psl::fract(val.x), psl::fract(val.y)};
}
template <typename T>
Vector3<T> fract(Vector3<T> val) {
  return {psl::fract(val.x), psl::fract(val.y), psl::fract(val.z)};
}
template <typename T>
Vector4<T> fract(Vector4<T> val) {
  return {psl::fract(val.x), psl::fract(val.y), psl::fract(val.z), psl::fract(val.w)};
}

template <typename T>
Vector2<T> floor(Vector2<T> val) {
  return {psl::floor(val.x), psl::floor(val.y)};
}
template <typename T>
Vector3<T> floor(Vector3<T> val) {
  return {psl::floor(val.x), psl::floor(val.y), psl::floor(val.z)};
}
template <typename T>
Vector4<T> floor(Vector4<T> val) {
  return {psl::floor(val.x), psl::floor(val.y), psl::floor(val.z), psl::floor(val.w)};
}

template <typename T>
Vector2<T> ceil(Vector2<T> val) {
  return {psl::ceil(val.x), psl::ceil(val.y)};
}
template <typename T>
Vector3<T> ceil(Vector3<T> val) {
  return {psl::ceil(val.x), psl::ceil(val.y), psl::ceil(val.z)};
}
template <typename T>
Vector4<T> ceil(Vector4<T> val) {
  return {psl::ceil(val.x), psl::ceil(val.y), psl::ceil(val.z), psl::ceil(val.w)};
}

template <typename T>
Vector2<T> sqrt(Vector2<T> val) {
  return {psl::sqrt(val.x), psl::sqrt(val.y)};
}
template <typename T>
Vector3<T> sqrt(Vector3<T> val) {
  return {psl::sqrt(val.x), psl::sqrt(val.y), psl::sqrt(val.z)};
}
template <typename T>
Vector4<T> sqrt(Vector4<T> val) {
  return {psl::sqrt(val.x), psl::sqrt(val.y), psl::sqrt(val.z), psl::sqrt(val.w)};
}

template <typename T>
Vector2<T> exp(Vector2<T> val) {
  return {psl::exp(val.x), psl::exp(val.y)};
}
template <typename T>
Vector3<T> exp(Vector3<T> val) {
  return {psl::exp(val.x), psl::exp(val.y), psl::exp(val.z)};
}
template <typename T>
Vector4<T> exp(Vector4<T> val) {
  return {psl::exp(val.x), psl::exp(val.y), psl::exp(val.z), psl::exp(val.w)};
}

template <typename T>
Vector2<T> pow(Vector2<T> val, T p) {
  return {psl::pow(val.x, p), psl::pow(val.y, p)};
}
template <typename T>
Vector3<T> pow(Vector3<T> val, T p) {
  return {psl::pow(val.x, p), psl::pow(val.y, p), psl::pow(val.z, p)};
}
template <typename T>
Vector4<T> pow(Vector4<T> val, T p) {
  return {psl::pow(val.x, p), psl::pow(val.y, p), psl::pow(val.z, p), psl::pow(val.w, p)};
}
template <typename T>
Vector2<T> pow(Vector2<T> val, Vector2<T> p) {
  return {psl::pow(val.x, p.x), psl::pow(val.y, p.y)};
}
template <typename T>
Vector3<T> pow(Vector3<T> val, Vector3<T> p) {
  return {psl::pow(val.x, p.x), psl::pow(val.y, p.y), psl::pow(val.z, p.z)};
}
template <typename T>
Vector4<T> pow(Vector4<T> val, Vector4<T> p) {
  return {psl::pow(val.x, p.x), psl::pow(val.y, p.y), psl::pow(val.z, p.z), psl::pow(val.w, p.w)};
}

template <typename T>
Vector2<T> abs(Vector2<T> val) {
  return {psl::abs(val.x), psl::abs(val.y)};
}
template <typename T>
Vector3<T> abs(Vector3<T> val) {
  return {psl::abs(val.x), psl::abs(val.y), psl::abs(val.z)};
}
template <typename T>
Vector4<T> abs(Vector4<T> val) {
  return {psl::abs(val.x), psl::abs(val.y), psl::abs(val.z), psl::abs(val.w)};
}

template <typename T>
bool inside(Vector2<T> p, Vector2<T> minInclude, Vector2<T> maxExclude) {
  return p.x >= minInclude.x && p.y >= minInclude.y && p.x < maxExclude.x && p.y < maxExclude.y;
}
template <typename T>
bool inside(Vector3<T> p, Vector3<T> minInclude, Vector3<T> maxExclude) {
  return p.x >= minInclude.x && p.x < maxExclude.x && p.y >= minInclude.y && p.y < maxExclude.y &&
         p.z >= minInclude.z && p.z < maxExclude.z;
}

template <typename T, typename U>
T trilinear_interp(T c[2][2][2], Vector3<U> uvw) {
  T ret = 0;
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++)
        ret += (x * uvw.x + (1 - x) * (1.0 - uvw.x)) * (y * uvw.y + (1 - y) * (1.0 - uvw.y)) *
               (z * uvw.z + (1 - z) * (1.0 - uvw.z)) * c[x][y][z];
  return ret;
}

template <typename T>
T perlin_interp(Vector3<T> c[2][2][2], Vector3<T> uvw) {
  T ret = 0;
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++) {
        Vector3<T> weight = uvw - Vector3<T>(x, y, z);
        ret += (x * uvw.x + (1 - x) * (1.0 - uvw.x)) * (y * uvw.y + (1 - y) * (1.0 - uvw.y)) *
               (z * uvw.z + (1 - z) * (1.0 - uvw.z)) * dot(c[x][y][z], weight);
      }
  return ret;
}

template <typename T>
Matrix2<T> transpose(const Matrix2<T> &m) {
  return {m.row(0), m.row(1)};
}
template <typename T>
Matrix3<T> transpose(const Matrix3<T> &m) {
  return {m.row(0), m.row(1), m.row(2)};
}
template <typename T>
Matrix4<T> transpose(const Matrix4<T> &m) {
  return {m.row(0), m.row(1), m.row(2), m.row(3)};
}

// Floating-point vecmath

inline float safe_rcp(float v) {
  return v == 0.0f ? 1e+20f : 1.0f / v;
}
inline vec3 safe_rcp(vec3 v) {
  v.x = v.x == 0.0f ? 1e+20f : 1.0f / v.x;
  v.y = v.y == 0.0f ? 1e+20f : 1.0f / v.y;
  v.z = v.z == 0.0f ? 1e+20f : 1.0f / v.z;
  return v;
}
inline vec4 safe_rcp(vec4 v) {
  v.x = v.x == 0.0f ? 1e+20f : 1.0f / v.x;
  v.y = v.y == 0.0f ? 1e+20f : 1.0f / v.y;
  v.z = v.z == 0.0f ? 1e+20f : 1.0f / v.z;
  v.w = v.w == 0.0f ? 1e+20f : 1.0f / v.w;
  return v;
}

inline float determinant(const mat3 &m) {
  return dot(m.x, cross(m.y, m.z));
}

inline mat2 inverse(const mat2 &m) {
  float d = m[0][0] * m[1][1] - m[1][0] * m[0][1];
  // clang-format off
    return mat2(
     m[1][1], -m[1][0], 
    -m[0][1],  m[0][0]
    ) / d;
  // clang-format on
}

vec3 solve(mat3 m, vec3 b);
mat3 inverse(const mat3 &m);
mat4 inverse(const mat4 &m);

inline mat4 translate(float x, float y, float z) {
  // clang-format off
  return {1.0f, 0.0f, 0.0f, x, 
          0.0f, 1.0f, 0.0f, y,
          0.0f, 0.0f, 1.0f, z, 
          0.0f, 0.0f, 0.0f, 1.0f};
  // clang-format on
}

inline mat4 translate(vec3 v) {
  // clang-format off
  return {1.0f, 0.0f, 0.0f, v.x, 
          0.0f, 1.0f, 0.0f, v.y,
          0.0f, 0.0f, 1.0f, v.z, 
          0.0f, 0.0f, 0.0f, 1.0f};
  // clang-format on
}

inline mat4 scale(float x, float y, float z) {
  // clang-format off
  return {x, 0.0f, 0.0f, 0.0f,
          0.0f, y, 0.0f,0.0f,
          0.0f, 0.0f, z,0.0f,
          0.0f,0.0f,0.0f,1.0f};
  // clang-format on
}

inline mat4 scale(vec3 v) {
  // clang-format off
  return {v.x, 0.0f, 0.0f,0.0f,
          0.0f, v.y, 0.0f,0.0f,
          0.0f, 0.0f, v.z,0.0f,
          0.0f,0.0f,0.0f,1.0f};
  // clang-format on
}

inline mat4 rotate_z(float rad) {
  // clang-format off
  return mat4{psl::cos(rad), -psl::sin(rad), 0, 0,
              psl::sin(rad), psl::cos(rad), 0, 0, 
              0, 0, 1, 0,
              0, 0, 0, 1
             };
  // clang-format on
}
inline mat4 rotate_x(float rad) {
  // clang-format off
  return mat4{1, 0, 0, 0,
              0, psl::cos(rad), -psl::sin(rad), 0, 
              0, psl::sin(rad), psl::cos(rad), 0,
              0, 0, 0, 1
             };
  // clang-format on
}
inline mat4 rotate_y(float rad) {
  // clang-format off
  return mat4{psl::cos(rad), 0, psl::sin(rad), 0,
              0, 1, 0, 0, 
              -psl::sin(rad), 0, psl::cos(rad), 0,
              0, 0, 0, 1
             };
  // clang-format on
}
inline mat4 rotate(vec3 r) {
  return rotate_x(r[0]) * rotate_y(r[1]) * rotate_z(r[2]);
}

inline mat4 look_at(vec3 from, vec3 at, vec3 up = vec3(0, 1, 0)) {
  vec3 z = normalize(at - from);

  if (psl::abs(dot(z, up)) > 0.999f)
    z = normalize(z + vec3(0.0f, 0.0f, 1e-5f));

  vec3 x = normalize(cross(up, z));
  vec3 y = cross(z, x);
  return mat4((vec4)x, (vec4)y, (vec4)z, vec4(from, 1.0f));
}

inline void coordinate_system(vec3 n, vec3 &t, vec3 &b) {
  if (psl::abs(n.x) > psl::abs(n.y))
    t = normalize(cross(n, vec3(0, 1, 0)));
  else
    t = normalize(cross(n, vec3(1, 0, 0)));
  b = cross(n, t);
}

inline mat3 coordinate_system(vec3 n) {
  mat3 m;
  m.z = n;
  coordinate_system(n, m.x, m.y);
  return m;
}

inline vec3 spherical_to_cartesian(float phi, float theta) {
  float sinTheta = psl::sin(theta);
  return vec3(sinTheta * psl::cos(phi), sinTheta * psl::sin(phi), psl::cos(theta));
}
inline vec3 unit_square_to_cartesian(vec2 sc) {
  return spherical_to_cartesian(sc[0] * Pi * 2, sc[1] * Pi);
}


// The following function is by Francesco Mazzoli
// detailed in article "Speeding up atan2f by 50x": https://mazzo.li/posts/vectorized-atan2.html
float atan2_approx(float y, float x);
inline float phi2pi(float x, float y) {
  // float phi = psl::atan2(y, x);
  float phi = atan2_approx(y, x);
  return phi < 0.0f ? Pi * 2 + phi : phi;
}

inline vec2 cartesian_to_spherical(vec3 d) {
  return vec2(phi2pi(d.x, d.y), psl::acos(d.z));
}
inline vec2 cartesian_to_unit_square(vec3 d) {
  return cartesian_to_spherical(d) / vec2{2 * Pi, Pi};
}

inline vec3 face_same_hemisphere(vec3 v, vec3 ref) {
  return dot(v, ref) < 0 ? -v : v;
}

inline uint32_t left_shift_32(uint32_t x) {
  if (x == (1 << 10))
    x--;
  x = (x | (x << 16)) & 0x30000ff;
  x = (x | (x << 8)) & 0x300f00f;
  x = (x | (x << 4)) & 0x30c30c3;
  x = (x | (x << 2)) & 0x9249249;
  return x;
}
inline uint64_t left_shift_64(uint64_t x) {
  uint64_t v = 0;
  for (int i = 0; i < 10; i++)
    v |= (1u << (i * 3)) & (x << (i * 2));
  return v;
}

inline uint32_t encode_morton_32x3(vec3 v) {
  constexpr int kMortonBits = 10;
  constexpr int kMortonScale = 1 << kMortonBits;
  v *= kMortonScale;
  return (left_shift_32(v.z) << 2) | (left_shift_32(v.y) << 1) | left_shift_32(v.x);
}
inline uint64_t encode_morton_64x3(vec3 v) {
  constexpr int kMortonBits = 20;
  constexpr int kMortonScale = 1 << kMortonBits;
  v *= kMortonScale;
  return (left_shift_64(v.z) << 2) | (left_shift_64(v.y) << 1) | left_shift_64(v.x);
}

template <typename T>
inline int min_axis(Vector2<T> v) {
  return v[0] < v[1] ? 0 : 1;
}
template <typename T>
inline int max_axis(Vector2<T> v) {
  return v[0] > v[1] ? 0 : 1;
}
template <typename T>
inline int min_axis(Vector3<T> v) {
  if (v[0] < v[1])
    return v[0] < v[2] ? 0 : 2;
  else
    return v[1] < v[2] ? 1 : 2;
}
template <typename T>
inline int max_axis(Vector3<T> v) {
  if (v[0] > v[1])
    return v[0] > v[2] ? 0 : 2;
  else
    return v[1] > v[2] ? 1 : 2;
}
template <typename T>
inline T max_value(Vector2<T> v) {
  return psl::max(v[0], v[1]);
}
template <typename T>
inline T min_value(Vector2<T> v) {
  return psl::min(v[0], v[1]);
}
template <typename T>
inline T max_value(Vector3<T> v) {
  return psl::max(v[0], v[1], v[2]);
}
template <typename T>
inline T min_value(Vector3<T> v) {
  return psl::min(v[0], v[1], v[2]);
}

void vecmath_context(Context &ctx);

}  // namespace pine
