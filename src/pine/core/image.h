#pragma once

#include <pine/core/array.h>
#include <pine/core/color.h>

#include <psl/variant.h>
#include <psl/memory.h>

namespace pine {

struct Image : psl::variant<Array2d<vec3u8>, Array2d<vec4u8>, Array2d<vec3>, Array2d<vec4>> {
  using variant::variant;

  vec4 operator[](vec2i p) const;
  vec4 filtered(vec2 p) const;

  vec2i size() const {
    return dispatch([](auto&& x) { return x.size(); });
  }

  template <typename T>
  T convert() const {
    return dispatch([](auto&& x) { return T::from(x); });
  }
};
using ImagePtr = psl::shared_ptr<Image>;

float mse(const Image& a, const Image& b);
float rmse(const Image& ref, const Image& b);

void image_context(Context& ctx);

}  // namespace pine