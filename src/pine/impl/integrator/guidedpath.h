#pragma once

#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

class GuidedPathIntegrator : public RTIntegrator {
public:
  GuidedPathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                       int max_path_length, int estimate_samples = 0)
      : RTIntegrator{psl::move(accel), psl::move(sampler)},
        light_sampler{psl::move(light_sampler)},
        max_path_length{max_path_length},
        estimate_samples{psl::max(estimate_samples, 0)},
        use_estimated_radiance{estimate_samples > 0} {
  }

  void render(Scene& scene) override;

  struct Vertex {
    Vertex(int length, vec3 n, vec3 p, float pdf, bool is_delta = false)
        : length(length), n(n), p(p), pdf(pdf), is_delta(is_delta) {
    }
    static Vertex first_vertex() {
      return Vertex(0, vec3(0), vec3(0), 0.0f, true);
    }
    int length;
    vec3 n;
    vec3 p;
    float pdf;
    bool is_delta;
  };
  struct RadianceResult {
    vec3 Lo;
    psl::optional<float> mis_term;
  };
  RadianceResult radiance(Scene& scene, Ray ray, Sampler& sampler, Vertex prev_vertex);

private:
  LightSampler light_sampler;
  int max_path_length;
  int estimate_samples;
  bool use_estimated_radiance;
  bool estimate_radiance;
  bool collect_radiance_sample;
  float use_learned_ratio;
};

}  // namespace pine