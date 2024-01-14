#pragma once

#include <pine/core/integrator.h>

namespace pine {

class AOIntegrator : public RayIntegrator {
public:
  using RayIntegrator::RayIntegrator;

  vec3 radiance(Scene& scene, Ray ray, Sampler& sampler) override;
};

}  // namespace pine
