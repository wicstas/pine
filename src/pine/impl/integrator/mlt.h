#pragma once
#include <pine/core/integrator.h>

namespace pine {

struct MltIntegrator : public RTIntegrator {
  MltIntegrator(Accel accel, int avg_spp, LightSampler light_sampler, int max_path_length);

  void render(Scene& scene) override;

  struct RadianceResult {
    vec3 Lo;
    psl::optional<float> light_pdf;
  };
  struct Vertex;
  RadianceResult radiance(Scene& scene, Ray ray, Sampler& sampler, Vertex pv, bool omit_direct) const;

private:
  int avg_spp;
  int max_path_length;
};

}  // namespace pine