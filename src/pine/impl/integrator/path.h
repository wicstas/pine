#pragma once

#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

struct PathIntegrator : public RayIntegrator {
  PathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler, int max_path_length);

  void render(Scene& scene) override;
  vec3 radiance(Scene& scene, Ray ray, Sampler& sampler) override;

private:
  LightSampler light_sampler;
  int max_path_length;
};

}  // namespace pine