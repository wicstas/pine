#pragma once

#include <pine/core/integrator.h>

namespace pine {

class VisualizerIntegrator : public RayIntegrator {
public:
  VisualizerIntegrator(Accel accel, Sampler sampler, psl::string viz_type);

  vec3 radiance(Scene& scene, Ray ray, Sampler& sampler) override;

  enum { Normal, Position, UV, BVH } viz_type;
};

}  // namespace pine
