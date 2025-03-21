#pragma once

#include <pine/core/integrator.h>

namespace pine {

struct RandomWalkIntegrator : public RayIntegrator {
  RandomWalkIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                       int max_path_length);

  vec3 radiance(Scene& scene, Ray ray, Sampler& sampler) override;

 private:
  int max_path_length;
};

}  // namespace pine