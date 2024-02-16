#pragma once

#include <pine/core/geometry.h>
#include <pine/core/sampler.h>
#include <pine/core/scene.h>
#include <pine/core/accel.h>
#include <pine/core/film.h>

#include <psl/function.h>

namespace pine {

void set_progress(float progress);
float get_progress();

class Integrator {
public:
  virtual ~Integrator() = default;

  virtual void render(Scene& scene) = 0;
};

class RTIntegrator : public Integrator {
public:
  RTIntegrator(Accel accel, Sampler sampler);

  bool hit(Ray ray) const {
    return accel.hit(ray);
  }
  uint8_t hit8(psl::span<const Ray> rays) const {
    return accel.hit8(rays);
  }
  bool intersect(Ray& ray, Interaction& it) const {
    return accel.intersect(ray, it);
  }

protected:
  Accel accel;
  psl::vector<Sampler> samplers;
  int samples_per_pixel;
  int primary_ratio = 8;
};

class RayIntegrator : public RTIntegrator {
public:
  using RTIntegrator::RTIntegrator;

  void render(Scene& scene) override;
  virtual vec3 radiance(Scene& scene, Ray ray, Interaction it, bool is_hit, Sampler& sampler) = 0;
};

class CustomRayIntegrator : public RayIntegrator {
public:
  CustomRayIntegrator(
      Accel accel, Sampler sampler,
      psl::function<vec3(CustomRayIntegrator&, Scene&, Ray, Interaction, bool, Sampler&)> radiance_)
      : RayIntegrator(psl::move(accel), psl::move(sampler)), radiance_(psl::move(radiance_)) {
  }

  vec3 radiance(Scene& scene, Ray ray, Interaction it, bool is_hit, Sampler& sampler) override {
    return radiance_(*this, scene, ray, it, is_hit, sampler);
  }

private:
  psl::function<vec3(CustomRayIntegrator&, Scene&, Ray, Interaction, bool, Sampler&)> radiance_;
};

}  // namespace pine
