#include <pine/impl/integrator/vol.h>
#include <pine/core/scene.h>

namespace pine
{

  VolIntegrator::VolIntegrator(Accel accel, Sampler sampler)
      : RayIntegrator{MOVE(accel), MOVE(sampler), UniformLightSampler()}
  {
  }

  const auto Lp = vec3(0, 1, 4);

  inline vec3 E1(vec3 x)
  {
    auto A = log((0.56146f / x + vec3(0.65f)) * (vec3(1) + x));
    auto B = psl::powi(x, 4) * exp(7.7f * x) * pow(vec3(2) + x, 3.7f);
    LOG(x, ((0.56146f / x + vec3(0.65f)) * (vec3(1) + x)).x);
    return pow(pow(A, -7.7f) + B, -0.13f);
  }

  vec3 VolIntegrator::radiance(Scene &, Ray ray, Sampler &sampler)
  {
    const auto sigma_s = vec3(1);
    const auto sigma_a = vec3(0);
    const auto sigma_t = sigma_a + sigma_s;

    auto b = -2 * dot(Lp - ray.o, ray.d);
    auto c = length_squared(Lp - ray.o);
    auto delta = b * b - 4 * c;
    auto t0 = (-b - psl::sqrt(-delta)) / 2;
    auto t1 = (-b + psl::sqrt(-delta)) / 2;
    auto I = (exp(-sigma_t * t0) * E1(-sigma_s * t0) - exp(-sigma_t * t1) * E1(-sigma_t * t1)) / (t0 - t1);
    return sigma_s / sigma_t * I;
  }

} // namespace pine