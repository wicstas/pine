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

  T *data() {
    return data_.data();
  }
  const T *data() const {
    return data_.data();
  }
  vec2i size() const {
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

private:
  vec2i size_;
  psl::vector<T> data_;
};

template <typename T>
struct Array3d {
  Array3d() = default;
  Array3d(vec3i64 size) : size_{size}, data_(volume(size)) {
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
  auto size = upper - lower;
  for (int y = 0; y < size[1]; y++)
    for (int x = 0; x < size[0]; x++)
      f(lower + vec2i(x, y));
}
template <typename F>
void for_3d(vec3i lower, vec3i upper, F f) {
  auto size = upper - lower;
  for (int z = 0; z < size[2]; z++)
    for (int y = 0; y < size[1]; y++)
      for (int x = 0; x < size[0]; x++)
        f(lower + vec3i(x, y, z));
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

void array2d_context(Context &context);

}  // namespace pine