#pragma once

#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

struct DiffuseFilteredPathIntegrator : public RTIntegrator {
  DiffuseFilteredPathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                                int max_path_length)
      : RTIntegrator{psl::move(accel), psl::move(sampler)},
        light_sampler{psl::move(light_sampler)},
        max_path_length{max_path_length} {
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
    vec3 li;
    psl::optional<float> mis;
  };
  RadianceResult radiance(Scene& scene, Ray ray, Interaction it, bool is_hit, Sampler& sampler,
                          Vertex pv) const;

private:
  LightSampler light_sampler;
  int max_path_length;
};

}  // namespace pine