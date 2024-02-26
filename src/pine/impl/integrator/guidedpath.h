#pragma once

#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

class GuidedPathIntegrator : public RTIntegrator {
public:
  GuidedPathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                       int max_path_length);

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
  vec3 radiance(Scene& scene, Ray ray, Interaction it, bool is_hit, Sampler& sampler,
                Vertex prev_vertex);

private:
  LightSampler light_sampler;
  int max_path_length;
  bool collect_radiance_sample;
  float use_learned_ratio;
  int primary_ratio = 8;
};

}  // namespace pine