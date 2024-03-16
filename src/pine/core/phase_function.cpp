#include <pine/core/phase_function.h>

namespace pine {

PhaseFunctionSample HgPhaseFunction::sample(vec3 wi, vec2 u) const {
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

}  // namespace pine