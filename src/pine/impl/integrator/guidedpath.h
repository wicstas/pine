#pragma once

#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

class GuidedPathIntegrator : public RTIntegrator {
public:
  GuidedPathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler, int max_depth,
                       int estimate_samples = 0)
      : RTIntegrator{psl::move(accel), psl::move(sampler)},
        light_sampler{psl::move(light_sampler)},
        max_depth{max_depth},
        estimate_samples{psl::max(estimate_samples, 0)},
        use_estimate{estimate_samples > 0} {
  }

  void render(Scene& scene) override;
  vec3 radiance_estimate(Scene& scene, Ray ray, Sampler& sampler, int depth);

  struct Vertex {
    int length = 0;
  };
  vec3 radiance(Scene& scene, Ray ray, Sampler& sampler, Vertex vertex);

private:
  LightSampler light_sampler;
  int max_depth;
  int estimate_samples;
  bool use_estimate;
};

}  // namespace pine