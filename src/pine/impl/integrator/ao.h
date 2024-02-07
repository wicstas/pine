#pragma once

#include <pine/core/integrator.h>

namespace pine {

class AOIntegrator : public RayIntegrator {
public:
  using RayIntegrator::RayIntegrator;

  void pre_render(Scene& scene) override {
    auto s = scene.get_aabb().diagonal();
    radius = psl::min(s.x, s.y, s.z) / 10;
  }
  vec3 radiance(Scene& scene, Ray ray, Sampler& sampler) override;

private:
  float radius = Infinity;
};

}  // namespace pine
