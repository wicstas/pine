#pragma once
#include <pine/core/lightsampler.h>
#include <pine/core/geometry.h>
#include <pine/core/sampler.h>
#include <pine/core/scene.h>
#include <pine/core/accel.h>
#include <pine/core/film.h>

#include <psl/function.h>

namespace pine {

inline float progress_2d(vec2 p, vec2i size) {
  return (p.x + p.y * size.x) / area(size);
}

void set_progress(float progress);
float get_progress();

class Integrator {
public:
  virtual ~Integrator() = default;

  virtual void render(Scene& scene) = 0;
};

class RTIntegrator : public Integrator {
public:
  RTIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler);

  bool hit(Ray ray) const {
    return accel.hit(ray);
  }
  uint8_t hit8(psl::span<const Ray> rays) const {
    return accel.hit8(rays);
  }
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  psl::optional<SurfaceInteraction> intersect(Ray& ray) const;
  psl::optional<MediumSample> sample_medium(Ray ray, Sampler& sampler) const;
  vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler,
                     int skip_medium_index = -1) const;

  void render(Scene& scene) override;

protected:
  const Scene* scene;
  Accel accel;
  LightSampler light_sampler;
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
  CustomRayIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                      psl::function<vec3(CustomRayIntegrator&, Ray, Sampler&)> radiance_)
      : RayIntegrator(MOVE(accel), MOVE(sampler), MOVE(light_sampler)), radiance_(MOVE(radiance_)) {
  }

  vec3 radiance(Scene&, Ray ray, Sampler& sampler) override {
    return radiance_(*this, ray, sampler);
  }

private:
  psl::function<vec3(CustomRayIntegrator&, Ray, Sampler&)> radiance_;
};

}  // namespace pine
