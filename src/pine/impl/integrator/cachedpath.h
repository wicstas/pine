#pragma once
#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

struct CachedPathIntegrator : public RTIntegrator {
  CachedPathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                       int max_path_length, int max_axis_resolution, int starting_depth = 2);
  void render(Scene& scene) override;

  struct RadianceResult {
    vec3 Lo;
    psl::optional<float> light_pdf;
  };
  struct Vertex;
  RadianceResult radiance(Scene& scene, Ray ray, Sampler& sampler, Vertex pv);

private:
  int max_path_length;
  int max_axis_resolution;
  int starting_depth;
  float footprint;
  bool learning_phase;
};

}  // namespace pine