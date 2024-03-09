#pragma once

#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

class EARSIntegrator : public RTIntegrator {
public:
  EARSIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler, int max_path_length);

  void render(Scene& scene) override;

  struct Vertex;
  struct RadianceResult {
    vec3 Lo;
    psl::optional<float> light_pdf;
    float cost = 0.0f;
  };
  struct Stats {
    vec3 value;
  };
  RadianceResult radiance(Scene& scene, Ray ray, Sampler& sampler, Vertex prev_vertex,
                          Stats& stats);

private:
  LightSampler light_sampler;
  int max_path_length;
};

}  // namespace pine