#pragma once

#include <pine/core/integrator.h>

namespace pine {

class AOIntegrator : public RayIntegrator {
public:
  AOIntegrator(Accel accel, Sampler sampler);

  void render(Scene& scene) override {
    radius = min_value(scene.get_aabb().diagonal()) / 2;
    RayIntegrator::render(scene);
  }
  vec3 radiance(Scene& scene, Ray ray, Interaction it, bool is_hit, Sampler& sampler) override;

private:
  float radius;
};

}  // namespace pine
