#pragma once

#include <pine/core/array.h>
#include <pine/core/color.h>

#include <psl/variant.h>

namespace pine {

struct Image : psl::variant<Array2d<vec3u8>, Array2d<vec3>, Array2d<vec4>> {
  using variant::variant;

  vec3 operator[](vec2i p) const {
    return dispatch([p](auto&& x) {
      auto value = x[p];
      if constexpr (psl::SameAs<decltype(value), vec3u8>)
        return correct_gamma(value / 255.0f);
      else
        return vec3(value);
    });
  }

  vec2i size() const {
    return dispatch([](auto&& x) { return x.size(); });
  }
};

float mse(const Image& a, const Image& b);
float rmse(const Image& ref, const Image& b);

void image_context(Context& ctx);

}  // namespace pine