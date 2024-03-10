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
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  Interaction intersect_tr(Ray& ray, Sampler& sampler) const;

  bool intersect_cases(Ray& ray, Sampler& sampler, auto f_escape, auto f_surface,
                       auto f_medium) const {
    auto it_tr = intersect_tr(ray, sampler);
    if (!it_tr.is_valid())
      return f_escape();
    else if (it_tr.is<SurfaceInteraction>())
      return f_surface(it_tr.as<SurfaceInteraction>());
    else
      return f_medium(it_tr.as<MediumInteraction>());
  }

  void render(Scene& scene) override;

protected:
  const Scene* scene;
  Accel accel;
  psl::vector<Sampler> samplers;
  int spp;
};

class RayIntegrator : public RTIntegrator {
public:
  using RTIntegrator::RTIntegrator;

  void render(Scene& scene) override;
  virtual vec3 radiance(Scene& scene, Ray ray, Sampler& sampler) = 0;
};

class CustomRayIntegrator : public RayIntegrator {
public:
  CustomRayIntegrator(Accel accel, Sampler sampler,
                      psl::function<vec3(CustomRayIntegrator&, Scene&, Ray, Sampler&)> radiance_)
      : RayIntegrator(psl::move(accel), psl::move(sampler)), radiance_(psl::move(radiance_)) {
  }

  vec3 radiance(Scene& scene, Ray ray, Sampler& sampler) override {
    return radiance_(*this, scene, ray, sampler);
  }

private:
  psl::function<vec3(CustomRayIntegrator&, Scene&, Ray, Sampler&)> radiance_;
};

}  // namespace pine
