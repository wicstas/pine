#pragma once
#include <pine/core/vecmath.h>
#include <pine/core/log.h>

namespace pine {

template <typename T>
struct Array2d {
  Array2d() = default;
  Array2d(vec2i size) : size_{size}, data_(area(size)) {
  }
  Array2d(vec2i size, const T *input) : size_{size}, data_(area(size)) {
    psl::memcpy(&data_[0], input, data_.size() * sizeof(T));
  }
  template <typename U>
  static Array2d from(const Array2d<U> &rhs) {
    auto result = Array2d(rhs.size());
    for (size_t i = 0; i < result.data_.size(); i++)
      result.data_[i] = T(rhs.data()[i]);
    return result;
  }

  T &operator[](vec2i p) {
    DCHECK_RANGE(p[0], 0, size_[0] - 1);
    DCHECK_RANGE(p[1], 0, size_[1] - 1);
    return data()[p[0] + p[1] * size_[0]];
  }
  const T &operator[](vec2i p) const {
    DCHECK_RANGE(p[0], 0, size_[0] - 1);
    DCHECK_RANGE(p[1], 0, size_[1] - 1);
    return data()[p[0] + p[1] * size_[0]];
  }
  Array2d &operator*=(auto rhs) {
    for (auto &x : (*this))
      x *= rhs;
    return *this;
  }
  Array2d &operator/=(auto rhs) {
    for (auto &x : (*this))
      x /= rhs;
    return *this;
  }
  Array2d &operator+=(auto rhs) {
    for (auto &x : (*this))
      x += rhs;
    return *this;
  }
  Array2d &operator-=(auto rhs) {
    for (auto &x : (*this))
      x -= rhs;
    return *this;
  }
  T &at_index(size_t i) {
    DCHECK_LT(i, data_.size());
    return data_[i];
  }
  const T &at_index(size_t i) const {
    DCHECK_LT(i, data_.size());
    return data_[i];
  }
  size_t index(vec2i p) const {
    return p[0] + p[1] * size_[0];
  }

  Array2d &operator+=(T value) {
    for (auto &x : data_)
      x += value;
    return *this;
  }
  Array2d &operator-=(T value) {
    for (auto &x : data_)
      x -= value;
    return *this;
  }
  Array2d &operator*=(T value) {
    for (auto &x : data_)
      x *= value;
    return *this;
  }
  Array2d &operator/=(T value) {
    for (auto &x : data_)
      x /= value;
    return *this;
  }
  friend Array2d operator+(Array2d lhs, T rhs) {
    return lhs += rhs;
  }
  friend Array2d operator-(Array2d lhs, T rhs) {
    return lhs -= rhs;
  }
  friend Array2d operator*(Array2d lhs, T rhs) {
    return lhs *= rhs;
  }
  friend Array2d operator/(Array2d lhs, T rhs) {
    return lhs /= rhs;
  }

  T *data() {
    return data_.data();
  }
  const T *data() const {
    return data_.data();
  }
  vec2i size() const {
    return size_;
  }
  int width() const {
    return size().x;
  }
  int height() const {
    return size().y;
  }

  auto begin() {
    return data_.begin();
  }
  auto end() {
    return data_.end();
  }
  auto begin() const {
    return data_.begin();
  }
  auto end() const {
    return data_.end();
  }
  void set_to_zero() {
    for (auto &val : data_)
      val = T();
  }

private:
  vec2i size_;
  psl::vector<T> data_;
};

template <typename T>
struct Array3d {
  Array3d() = default;
  Array3d(vec3i64 size) : size_{size}, data_(volume(size)) {
  }
  template <typename U>
  static Array3d from(const Array3d<U> &rhs) {
    auto result = Array3d(rhs.size());
    for (size_t i = 0; i < result.data_.size(); i++)
      result.data_[i] = T(rhs.data()[i]);
    return result;
  }

  T &operator[](vec3i64 p) {
    DCHECK_RANGE(p[0], 0, size_[0] - 1);
    DCHECK_RANGE(p[1], 0, size_[1] - 1);
    DCHECK_RANGE(p[2], 0, size_[2] - 1);
    return data()[p[0] + p[1] * size_[0] + p[2] * size_[0] * size_[1]];
  }
  const T &operator[](vec3i64 p) const {
    DCHECK_RANGE(p[0], 0, size_[0] - 1);
    DCHECK_RANGE(p[1], 0, size_[1] - 1);
    DCHECK_RANGE(p[2], 0, size_[2] - 1);
    return data()[p[0] + p[1] * size_[0] + p[2] * size_[0] * size_[1]];
  }
  T &at_index(size_t i) {
    DCHECK_LT(i, data_.size());
    return data_[i];
  }
  const T &at_index(size_t i) const {
    DCHECK_LT(i, data_.size());
    return data_[i];
  }
  size_t index(vec3i64 p) const {
    return p[0] + p[1] * size_[0] + p[2] * size_[0] * size_[1];
  }

  Array3d &operator+=(T value) {
    for (auto &x : data_)
      x += value;
    return *this;
  }
  Array3d &operator-=(T value) {
    for (auto &x : data_)
      x -= value;
    return *this;
  }
  Array3d &operator*=(T value) {
    for (auto &x : data_)
      x *= value;
    return *this;
  }
  Array3d &operator/=(T value) {
    for (auto &x : data_)
      x /= value;
    return *this;
  }

  T *data() {
    return data_.data();
  }
  const T *data() const {
    return data_.data();
  }
  vec3i64 size() const {
    return size_;
  }

  auto begin() {
    return data_.begin();
  }
  auto end() {
    return data_.end();
  }
  auto begin() const {
    return data_.begin();
  }
  auto end() const {
    return data_.end();
  }
  void set_to_zero() {
    for (auto &val : data_)
      val = T();
  }

private:
  vec3i64 size_;
  psl::vector<T> data_;
};

using Array2df = Array2d<float>;
using Array2d2f = Array2d<vec2>;
using Array2d3f = Array2d<vec3>;
using Array2d4f = Array2d<vec4>;

template <typename F>
void for_2d(vec2i lower, vec2i upper, F f) {
  for (int y = lower[1]; y < upper[1]; y++)
    for (int x = lower[0]; x < upper[0]; x++)
      f(vec2i(x, y));
}
template <typename F>
void for_3d(vec3i lower, vec3i upper, F f) {
  for (int z = lower[2]; z < upper[2]; z++)
    for (int y = lower[1]; y < upper[1]; y++)
      for (int x = lower[0]; x < upper[0]; x++)
        f(vec3i(x, y, z));
}

template <typename F>
void for_2d(vec2i size, F f) {
  return for_2d(vec2i(), size, f);
}
template <typename F>
void for_3d(vec3i size, F f) {
  return for_3d(vec3i(), size, f);
}

inline Array2d2f grid(vec2 a, vec2 b, vec2i size) {
  auto arr = Array2d2f(size);
  for_2d(size, [&](vec2i p) { arr[p] = lerp((p + vec2(0.5f)) / size, a, b); });
  return arr;
}

template <typename T>
void combine_inplace(Array2d<T> &a, const Array2d<T> &b, float weight_a, float weight_b) {
  auto index = size_t{0};
  CHECK_EQ(a.size(), b.size());
  CHECK(weight_a + weight_b != 0.0f);
  auto inv_weight_sum = 1.0f / (weight_a + weight_b);
  for (auto &ea : a) {
    ea = (weight_a * ea + weight_b * b.data()[index]) * inv_weight_sum;
    index++;
  }
}
template <typename T>
Array2d<T> combine(Array2d<T> a, const Array2d<T> &b, float weight_a, float weight_b) {
  combine_inplace(a, b, weight_a, weight_b);
  return a;
}

void array2d_context(Context &context);

}  // namespace pine