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
  PhaseFunctionSample sample(vec3 wi, vec2 u) const {
    auto ps = PhaseFunctionSample();
    auto cos_theta = 0.0f;
    if (std::abs(g) < 1e-3f)
      cos_theta = 1 - 2 * u[0];
    else
      cos_theta = -1 / (2 * g) * (1 + sqr(g) - sqr((1 - sqr(g)) / (1 + g - 2 * g * u[0])));

    auto sin_theta = psl::safe_sqrt(1 - sqr(cos_theta));
    auto phi = 2 * Pi * u[1];
    auto m = coordinate_system(wi);
    ps.wo = m * spherical_to_cartesian(phi, sin_theta, cos_theta);
    ps.f = ps.pdf = henyey_greenstein(dot(wi, ps.wo), g);
    return ps;
  }
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

struct MediumSample {
  float t;
  PhaseFunction pg;
};

struct HomogeneousMedium {
  MediumSample sample(float u) const {
    return {-psl::log(u) / density, {}};
  }

  float density;
};

struct Medium : psl::variant<HomogeneousMedium> {
  MediumSample sample(float u) const {
    return dispatch([&](auto&& x) { return x.sample(u); });
  }
};

void medium_context(Context& context);

}  // namespace pine
