#pragma once

#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

class OpenPGLIntegrator : public RTIntegrator {
public:
  OpenPGLIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler, int max_depth)
      : RTIntegrator{psl::move(accel), psl::move(sampler)},
        light_sampler{psl::move(light_sampler)},
        max_depth{max_depth} {
  }

  struct RadianceResult {
    vec3 L;
    float distance;
  };

  void render(Scene& scene) override;
  RadianceResult radiance(Scene& scene, Ray ray, Sampler& sampler, int depth, float prev_sample_pdf,
                          vec3 prev_n, bool prev_delta);

private:
  LightSampler light_sampler;
  int max_depth;
};

}  // namespace pine