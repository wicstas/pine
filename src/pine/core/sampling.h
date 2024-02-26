#pragma once

#include <pine/core/vecmath.h>

namespace pine {

struct SpatialPdf {
  operator float() const {
    return pos * dir;
  }
  float pos = 0.0f;
  float dir = 0.0f;
};

inline vec2 sample_disk_polar(vec2 u) {
  float r = psl::sqrt(u[0]);
  float theta = 2 * Pi * u[1];
  return {r * psl::cos(theta), r * psl::sin(theta)};
}

inline vec2 sample_disk_concentric(vec2 u) {
  u = vec2(u.x * 2 - 1.0f, u.y * 2 - 1.0f);
  float theta, r;
  if (psl::abs(u.x) > psl::abs(u.y)) {
    r = u.x;
    theta = Pi / 4.0f * u.y / u.x;
  } else {
    r = u.y;
    theta = Pi / 2.0f - Pi / 4.0f * (u.x / u.y);
  }
  return r * vec2(psl::cos(theta), psl::sin(theta));
}

inline vec3 cosine_weighted_hemisphere(vec2 u) {
  auto d = sample_disk_concentric(u);
  auto z = psl::sqrt(psl::max(1.0f - d.x * d.x - d.y * d.y, 0.0f));
  return vec3(d.x, d.y, z);
}

inline vec3 uniform_sphere(vec2 u) {
  const auto phi = u.x * Pi * 2;
  const auto cos_theta = 1 - 2 * u.y;
  const auto sin_theta = psl::sqrt(1.0f - psl::sqr(cos_theta));
  return vec3(sin_theta * psl::cos(phi), sin_theta * psl::sin(phi), cos_theta);
}
inline vec2 inverse_uniform_sphere(vec3 d) {
  return vec2(phi2pi(d.x, d.y) / Pi2, (1 - d.z) / 2);
}

inline vec3 uniform_hemisphere(vec2 u) {
  const auto phi = u.x * Pi * 2;
  const auto cos_theta = u.y;
  const auto sin_theta = psl::sqrt(1.0f - psl::sqr(cos_theta));
  return vec3(sin_theta * psl::cos(phi), sin_theta * psl::sin(phi), cos_theta);
}
inline vec2 inverse_uniform_hemisphere(vec3 d) {
  return vec2(phi2pi(d.x, d.y) / Pi2, psl::acos(d.z));
}

inline float balance_heuristic(float aF, float pF, float aG, float pG) {
  return aF * pF / (aF * pF + aG * pG);
}
inline float balance_heuristic(float pF, float pG) {
  return pF / (pF + pG);
}

}  // namespace pine
