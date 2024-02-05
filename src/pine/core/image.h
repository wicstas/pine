#pragma once

#include <pine/core/array.h>

#include <psl/variant.h>

namespace pine {

struct Image : psl::variant<Array2d<vec3u8>, Array2d<vec3>> {
  using variant::variant;

  vec3 operator[](vec2i p) const {
    return dispatch([p](auto&& x) {
      auto value = x[p];
      if constexpr (psl::SameAs<decltype(value), vec3>)
        return value;
      else
        return pow(value / 255.0f, 2.2f);
    });
  }

  vec2i size() const {
    return dispatch([](auto&& x) { return x.size(); });
  }
};

void image_context(Context& ctx);

}  // namespace pine