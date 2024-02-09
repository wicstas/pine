#pragma once

#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

struct SingleBounceIntegrator : public RTIntegrator {
  SingleBounceIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler)
      : RTIntegrator{psl::move(accel), psl::move(sampler)},
        light_sampler{psl::move(light_sampler)} {
  }

  void render(Scene& scene);
  void pixel_color(Scene& scene, vec2i p, Sampler& sampler);
  vec3 radiance(Scene& scene, Ray ray, Sampler& sampler, bool indirect_bounce);

private:
  LightSampler light_sampler;
  int primary_ratio = 8;
};

}  // namespace pine