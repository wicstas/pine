#pragma once

#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>

namespace pine {

class EARSIntegrator : public RTIntegrator {
public:
  EARSIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler, int max_path_length);

  void render(Scene& scene) override;

  struct Vertex {
    Vertex(int iteration, int length, vec3 throughput, vec3 n, vec3 p, float pdf,
           bool is_delta = false)
        : iteration(iteration),
          length(length),
          throughput(throughput),
          n(n),
          p(p),
          pdf(pdf),
          is_delta(is_delta) {
    }
    static Vertex first_vertex(int iteration, vec3 I) {
      return Vertex(iteration, 0, vec3(1.0f) / I, vec3(0), vec3(0), 0.0f, true);
    }
    int iteration;
    int length;
    vec3 throughput;
    vec3 n;
    vec3 p;
    float pdf;
    bool is_delta;
  };
  struct RadianceResult {
    vec3 Lr;
    psl::optional<float> light_pdf;
    float cost;
  };
  RadianceResult radiance(Scene& scene, Ray ray, Sampler& sampler, Vertex prev_vertex);

private:
  LightSampler light_sampler;
  int max_path_length;
};

}  // namespace pine