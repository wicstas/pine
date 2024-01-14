#pragma once

#include <pine/core/vecmath.h>
#include <pine/core/log.h>

namespace pine {

template <typename T>
struct Array2D {
  Array2D() = default;
  Array2D(vec2i size) : size_{size}, data_{static_cast<size_t>(area(size))} {
  }
  Array2D(vec2i size, const T *input) : size_{size}, data_{static_cast<size_t>(area(size))} {
    psl::memcpy(&data_[0], input, data_.size() * sizeof(T));
  }

  T &operator[](vec2i p) {
    CHECK_RANGE(p[0], 0, size_[0] - 1);
    CHECK_RANGE(p[1], 0, size_[1] - 1);
    return data()[p[0] + p[1] * size_[0]];
  }
  const T &operator[](vec2i p) const {
    CHECK_RANGE(p[0], 0, size_[0] - 1);
    CHECK_RANGE(p[1], 0, size_[1] - 1);
    return data()[p[0] + p[1] * size_[0]];
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

using Array2Df = Array2D<vec2>;

template <typename F>
void for_2d(vec2i lower, vec2i upper, F f, bool horizontal = false) {
  auto size = upper - lower;
  if (horizontal)
    for (int i = 0; i < size[0]; i++)
      for (int j = 0; j < size[1]; j++) {
        if constexpr (psl::IsVoid<decltype(f(vec2i{}))>)
          f(lower + vec2i{i, j});
        else if (!f(lower + vec2i{i, j}))
          return;
      }
  else
    for (int j = 0; j < size[1]; j++)
      for (int i = 0; i < size[0]; i++) {
        if constexpr (psl::IsVoid<decltype(f(vec2i{}))>)
          f(lower + vec2i{i, j});
        else if (!f(lower + vec2i{i, j}))
          return;
      }
}

template <typename F>
void for_2d(vec2i size, F f, bool horizontal = false) {
  return for_2d(vec2i{}, size, f, horizontal);
}

inline Array2Df grid(vec2 a, vec2 b, vec2i size) {
  auto arr = Array2Df{size};
  for_2d(size, [&](vec2i p) { arr[p] = lerp((p + vec2{0.5f}) / size, a, b); });
  return arr;
}

}  // namespace pine