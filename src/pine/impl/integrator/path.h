#pragma once
#include <pine/core/integrator.h>

namespace pine {

struct PathIntegrator : public RTIntegrator {
  PathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler, int max_path_length);

  void render(Scene& scene) override;

  struct RadianceResult {
    vec3 Lo;
    psl::optional<float> light_pdf;
  };
  struct Vertex;
  RadianceResult radiance(Scene& scene, Ray ray, Sampler& sampler, Vertex pv);

private:
  int max_path_length;
};

}  // namespace pine