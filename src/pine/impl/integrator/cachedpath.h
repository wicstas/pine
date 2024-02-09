#pragma once

#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

struct CachedPathIntegrator : public RTIntegrator {
  CachedPathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler, int max_depth,
                       int max_axis_resolution, int starting_depth = 1)
      : RTIntegrator{psl::move(accel), psl::move(sampler)},
        light_sampler{psl::move(light_sampler)},
        max_depth{max_depth},
        max_axis_resolution(max_axis_resolution),
        starting_depth{starting_depth} {
  }

  void render(Scene& scene) override;

  struct Vertex {
    vec3 n;
    bool bsdf_is_delta = true;
    float sample_pdf = 0.0f;
    int non_delta_path_length = 0;
  };

  vec3 radiance(Scene& scene, Ray ray, Sampler& sampler, int depth, Vertex vertex, int ssp);

private:
  LightSampler light_sampler;
  int max_depth;
  int max_axis_resolution;
  int starting_depth;
  int primary_ratio = 8;
};

}  // namespace pine