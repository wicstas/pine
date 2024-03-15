#pragma once
#include <pine/core/vecmath.h>
#include <pine/core/aabb.h>

#include <psl/function.h>
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

struct MediumSample {
  float t;
  vec3 p;
  float tr;
  float pdf;
  float sigma;
  PhaseFunction pg;
};

struct HomogeneousMedium {
  HomogeneousMedium(AABB aabb, float sigma_a, float sigma_s)
      : aabb(aabb), sigma_s(sigma_s), sigma_z(sigma_a + sigma_s) {
  }

  psl::optional<MediumSample> sample(vec3 p, vec3 d, float tmax, Sampler& sampler) const;
  vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const;

private:
  AABB aabb;
  float sigma_s;
  float sigma_z;
};

struct VDBMedium {
  VDBMedium(psl::string filename, mat4 transform, float sigma_s, float sigma_z);
  psl::optional<MediumSample> sample(vec3 p, vec3 d, float tmax, Sampler& sampler) const;
  vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const;
  float density(vec3 p) const;

private:
  psl::opaque_shared_ptr handle;
  void* grid;
  mat4 transform;
  AABB aabb;
  float sigma_maj;
  float sigma_maj_inv;
  float sigma_s;
  float sigma_z;
  vec3 index_start;
  vec3 index_end;
};

struct Medium : psl::variant<HomogeneousMedium, VDBMedium> {
  psl::optional<MediumSample> sample(vec3 p, vec3 d, float tmax, Sampler& sampler) const {
    auto ms = dispatch([&](auto&& x) { return x.sample(p, d, tmax, sampler); });
    if (ms)
      ms->p = p + d * ms->t;
    return ms;
  }
  vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const {
    return dispatch([&](auto&& x) { return x.transmittance(p, d, tmax, sampler); });
  }
};

void medium_context(Context& context);

}  // namespace pine
