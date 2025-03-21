#pragma once
#include <pine/impl/integrator/spatial_tree.h>
#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

class GuidedPathIntegrator : public RTIntegrator {
public:
  GuidedPathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                       int max_path_length);

  void render(Scene& scene) override;

  struct Vertex;
  vec3 radiance(Scene& scene, Ray ray, Sampler& sampler, Vertex prev_vertex);

private:
SpatialTree guide;
  int max_path_length;
  bool collect_radiance_sample;
  float use_learned_ratio;
};

}  // namespace pine