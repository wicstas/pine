#pragma once

#include <pine/core/integrator.h>

namespace pine
{

  class VolIntegrator : public RayIntegrator
  {
  public:
    VolIntegrator(Accel accel, Sampler sampler);

    vec3 radiance(Scene &scene, Ray ray, Sampler &sampler) override;
  };

} // namespace pine
