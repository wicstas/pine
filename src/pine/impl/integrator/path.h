#pragma once

#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

struct PathIntegrator : public RayIntegrator {
  PathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler, int max_path_length)
      : RayIntegrator{psl::move(accel), psl::move(sampler)},
        light_sampler{psl::move(light_sampler)},
        max_path_length{max_path_length} {
  }

  void render(Scene& scene) override {
    light_sampler.build(&scene);
    RayIntegrator::render(scene);
  }
  vec3 radiance(Scene& scene, Ray ray, Interaction it, bool is_hit, Sampler& sampler) override;

private:
  LightSampler light_sampler;
  int max_path_length;
};

}  // namespace pine