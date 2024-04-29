#pragma once
#include <pine/core/vecmath.h>

#include <psl/variant.h>

namespace pine {

struct PhaseFunctionSample {
  vec3 wo;
  float f;
  float pdf;
};

float eval_henyey_greenstein(float cos_theta, float g);
float eval_draine(float u, float g, float a);

float sample_henyey_greenstein(float xi, float g);
float sample_draine_cos(float xi, float g, float a);

struct HgPhaseFunction {
  HgPhaseFunction(float g = 0.0f) : g(g) {
  }

  PhaseFunctionSample sample(vec3 wi, vec2 u) const;
  float f(vec3 wi, vec3 wo) const {
    return eval_henyey_greenstein(dot(wi, wo), g);
  }
  float pdf(vec3 wi, vec3 wo) const {
    return f(wi, wo);
  }

private:
  float g;
};

struct CloudPhaseFunction {
  CloudPhaseFunction(float d = 10.0f);

  PhaseFunctionSample sample(vec3 wi, vec2 u) const;
  float f(vec3 wi, vec3 wo) const {
    auto cos = dot(wi, wo);
    return psl::lerp(w, eval_henyey_greenstein(cos, g_hg), eval_draine(cos, g_d, a));
  }
  float pdf(vec3 wi, vec3 wo) const {
    return f(wi, wo);
  }

private:
  float g_hg, g_d;
  float a;
  float w;
};

struct PhaseFunction : psl::variant<HgPhaseFunction, CloudPhaseFunction> {
  using variant::variant;

  PhaseFunctionSample sample(vec3 wi, vec2 u) const {
    return dispatch([&](auto&& x) { return x.sample(-wi, u); });
  }
  float f(vec3 wi, vec3 wo) const {
    return dispatch([&](auto&& x) { return x.f(-wi, wo); });
  }
  float pdf(vec3 wi, vec3 wo) const {
    return dispatch([&](auto&& x) { return x.pdf(-wi, wo); });
  }
};

}  // namespace pine