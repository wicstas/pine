#pragma once

#include <pine/core/vecmath.h>

namespace pine {

struct Ray {
  Ray() = default;
  Ray(vec3 o, vec3 d) : o(o), d(d), tmin(0.0f), tmax(float_max){};
  Ray(vec3 o, vec3 d, float tmin, float tmax) : o(o), d(d), tmin(tmin), tmax(tmax){};

  vec3 operator()() const {
    return o + tmax * d;
  }
  vec3 operator()(float t) const {
    return o + t * d;
  }

  psl::string to_string() const {
    return psl::to_string("Ray{o=", o, ", d=", d, ", tmin=", tmin, ", tmax=", tmax, "}");
  }

  vec3 o;
  vec3 d;
  float tmin = 0.0f;
  float tmax = float_max;
};

inline vec3 offset_ray_origin(vec3 p, vec3 n) {
  auto origin = 1.0f / 32.0f;
  auto floatScale = 1.0f / 65536.0f;
  auto intScale = 256.0f;
  auto of_i = vec3i(intScale * n);
  auto p_i = psl::bitcast<vec3>(psl::bitcast<vec3i>(p) + vec3i(p.x < 0 ? -of_i.x : of_i.x,
                                                               p.y < 0 ? -of_i.y : of_i.y,
                                                               p.z < 0 ? -of_i.z : of_i.z));
  return {psl::abs(p.x) < origin ? p.x + n.x * floatScale : p_i.x,
          psl::abs(p.y) < origin ? p.y + n.y * floatScale : p_i.y,
          psl::abs(p.z) < origin ? p.z + n.z * floatScale : p_i.z};
}

}  // namespace pine
