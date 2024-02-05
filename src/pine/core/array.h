#pragma once

#include <pine/core/vecmath.h>
#include <pine/core/log.h>

namespace pine {

template <typename T>
struct Array2d {
  Array2d() = default;
  Array2d(vec2i size) : size_{size}, data_{static_cast<size_t>(area(size))} {
  }
  Array2d(vec2i size, const T *input) : size_{size}, data_{static_cast<size_t>(area(size))} {
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

using Array2df = Array2d<float>;
using Array2d2f = Array2d<vec2>;
using Array2d3f = Array2d<vec3>;
using Array2d4f = Array2d<vec4>;

template <typename F>
void for_2d(vec2i lower, vec2i upper, F f) {
  auto size = upper - lower;
  for (int j = 0; j < size[1]; j++)
    for (int i = 0; i < size[0]; i++) {
      if constexpr (psl::IsVoid<psl::ReturnType<F, vec2i>>)
        f(lower + vec2i{i, j});
      else if (!f(lower + vec2i{i, j}))
        return;
    }
}

template <typename F>
void for_2d(vec2i size, F f) {
  return for_2d(vec2i(), size, f);
}

inline Array2d2f grid(vec2 a, vec2 b, vec2i size) {
  auto arr = Array2d2f(size);
  for_2d(size, [&](vec2i p) { arr[p] = lerp((p + vec2(0.5f)) / size, a, b); });
  return arr;
}

void array2d_context(Context &context);

}  // namespace pine