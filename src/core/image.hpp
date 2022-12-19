#pragma once
#include <core/math.hpp>

#include <vector>

namespace pine {

template <typename T>
struct Image {
  Image() = default;
  Image(vec2i size) : size_(size) {
    CHECK_GE(size[0], 0);
    CHECK_GE(size[1], 0);
    data_.resize(nPixels());
  }
  template <typename B>
  Image(Image<B> b) : size_(b.size()), data_(b.nPixels()) {
    for (int i = 0; i < b.nPixels(); ++i) data()[i] = T(b.data()[i]);
  }

  vec2i size() const { return size_; }
  int width() const { return size()[0]; }
  int height() const { return size()[1]; }
  int nPixels() const { return width() * height(); }
  T* data() { return data_.data(); }
  const T* data() const { return data_.data(); }

  T& operator[](vec2i p) {
    DCHECK_GE(p[0], 0);
    DCHECK_GE(p[1], 0);
    DCHECK_LT(p[0], width());
    DCHECK_LT(p[1], height());
    return data()[p[0] + (height() - p[1] - 1) * width()];
  }
  const T& operator[](vec2i p) const {
    DCHECK_GE(p[0], 0);
    DCHECK_GE(p[1], 0);
    DCHECK_LT(p[0], width());
    DCHECK_LT(p[1], height());
    return data()[p[0] + (height() - p[1] - 1) * width()];
  }

  Image operator()(auto f) const {
    auto copy = *this;
    for (int i = 0; i < nPixels(); ++i) copy.data()[i] = f(data()[i]);
    return copy;
  }

  Image& operator*=(auto multiplier) {
    return *this = (*this)([&](auto color) { return color * multiplier; });
  }
  Image& operator/=(auto divisor) {
    return *this = (*this)([&](auto color) { return color / divisor; });
  }

 private:
  vec2i size_;
  std::vector<T> data_;
};

template <typename T>
Image<T> operator*(const Image<T>& image, T multiplier) {
  return image([&](auto color) { return color * multiplier; });
}
template <typename T>
Image<T> operator/(const Image<T>& image, T divisor) {
  return image([&](auto color) { return color / divisor; });
}

using ImageLDR = Image<vec3u8>;

void writeImageAsTGA(std::string fileName, const ImageLDR& image);
inline void writeImageAsTGA(std::string fileName, const Image<vec3>& image) {
  return writeImageAsTGA(
      fileName, ImageLDR(image([](auto x) {
                           return clamp(x, vec3(0), vec3(OneMinusEpsilon));
                         }) *
                         vec3(256.0f)));
}

}  // namespace pine
