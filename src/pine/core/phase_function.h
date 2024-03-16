#pragma once
#include <pine/core/vecmath.h>

#include <psl/variant.h>

namespace pine {

struct PhaseFunctionSample {
  vec3 wo;
  float f;
  float pdf;
};

inline float henyey_greenstein(float cos_theta, float g) {
  auto denom = 1 + sqr(g) + 2 * g * cos_theta;
  return (1 - sqr(g)) / (denom * psl::safe_sqrt(denom) * Pi * 4);
}

struct HgPhaseFunction {
  HgPhaseFunction(float g) : g(g) {
  }
  PhaseFunctionSample sample(vec3 wi, vec2 u) const;
  float f(vec3 wi, vec3 wo) const {
    return henyey_greenstein(dot(wi, wo), g);
  }
  float pdf(vec3 wi, vec3 wo) const {
    return f(wi, wo);
  }

private:
  float g;
};

struct PhaseFunction : psl::variant<HgPhaseFunction> {
  using variant::variant;

  PhaseFunctionSample sample(vec3 wi, vec2 u) const {
    return dispatch([&](auto&& x) { return x.sample(wi, u); });
  }
  float f(vec3 wi, vec3 wo) const {
    return dispatch([&](auto&& x) { return x.f(wi, wo); });
  }
  float pdf(vec3 wi, vec3 wo) const {
    return dispatch([&](auto&& x) { return x.pdf(wi, wo); });
  }
};

}  // namespace pine