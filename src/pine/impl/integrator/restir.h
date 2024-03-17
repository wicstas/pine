#pragma once
#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

struct RestirIntegrator : public RTIntegrator {
  RestirIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler, int max_path_length);

  void render(Scene& scene) override;

  struct RadianceResult {
    vec3 Lo;
    psl::optional<float> light_pdf;
  };
  struct Vertex;
  RadianceResult radiance(Scene& scene, Ray ray, Sampler& sampler, Vertex pv);

private:
  LightSampler light_sampler;
  int max_path_length;
};

}  // namespace pine