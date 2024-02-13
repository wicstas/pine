#pragma once

#include <pine/core/integrator.h>

namespace pine {

struct RandomWalkIntegrator : public RayIntegrator {
  RandomWalkIntegrator(Accel accel, Sampler sampler, int max_depth)
      : RayIntegrator{psl::move(accel), psl::move(sampler)}, max_depth{max_depth} {
  }

  vec3 radiance(Scene& scene, Ray ray, Interaction it, bool is_hit, Sampler& sampler) override;

private:
  int max_depth;
};

}  // namespace pine