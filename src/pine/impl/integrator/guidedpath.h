#pragma once

#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

class GuidedPathIntegrator : public RTIntegrator {
public:
  GuidedPathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                       int max_path_length);

  void render(Scene& scene) override;

  struct Vertex;
  struct RadianceResult {
    vec3 Lo;
    float cost = 0.0f;
  };
  struct Stats {
    vec3 value;
  };
  RadianceResult radiance(Scene& scene, Ray ray, Sampler& sampler, Vertex prev_vertex, Stats& stats);

private:
  LightSampler light_sampler;
  int max_path_length;
  bool collect_radiance_sample;
  float use_learned_ratio;
};

}  // namespace pine