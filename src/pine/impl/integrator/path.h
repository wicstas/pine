#pragma once

#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

struct PathIntegrator : public RayIntegrator {
  PathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler, int max_depth,
                 bool nee_env_light = true)
      : RayIntegrator{psl::move(accel), psl::move(sampler)},
        light_sampler{psl::move(light_sampler)},
        max_depth{max_depth},
        nee_env_light{nee_env_light} {
  }

  void render(Scene& scene) override {
    light_sampler.build(&scene);
    RayIntegrator::render(scene);
  }
  vec3 radiance(Scene& scene, Ray ray, Sampler& sampler) override;

private:
  LightSampler light_sampler;
  int max_depth;
  bool nee_env_light;
};

}  // namespace pine