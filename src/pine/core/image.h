#pragma once

#include <pine/core/array.h>

#include <pine/psl/variant.h>

namespace pine {

struct Image : psl::Variant<Array2D<vec3u8>, Array2D<vec3>> {
  using Variant::Variant;

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

}  // namespace pine