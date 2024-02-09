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

  bool hit(Ray ray) const;
  bool intersect(Ray& ray, Interaction& it) const;

protected:
  Accel accel;
  psl::vector<Sampler> samplers;
  int samplesPerPixel;
};

class PixelIntegrator : public RTIntegrator {
public:
  using RTIntegrator::RTIntegrator;

  void render(Scene& scene) override;
  virtual void pixel_color(Scene& scene, vec2i p, Sampler& sampler) = 0;
  virtual void pre_render(Scene&) {
  }
};

class RayIntegrator : public PixelIntegrator {
public:
  using PixelIntegrator::PixelIntegrator;

  void pixel_color(Scene& scene, vec2i p, Sampler& sampler) override;
  virtual vec3 radiance(Scene& scene, Ray ray, Sampler& sampler) = 0;
};

class CustomRayIntegrator : public RayIntegrator {
public:
  CustomRayIntegrator(Accel accel, Sampler sampler,
                      psl::function<vec3, CustomRayIntegrator&, Scene&, Ray, Sampler&> radiance_)
      : RayIntegrator(psl::move(accel), psl::move(sampler)), radiance_(psl::move(radiance_)) {
  }

  vec3 radiance(Scene& scene, Ray ray, Sampler& sampler) override {
    return radiance_(*this, scene, ray, sampler);
  }

private:
  psl::function<vec3, CustomRayIntegrator&, Scene&, Ray, Sampler&> radiance_;
};

}  // namespace pine
