#pragma once
#include <pine/core/light.h>

#include <psl/optional.h>
#include <psl/variant.h>
#include <psl/vector.h>

namespace pine {

struct UniformLightSampler {
  void build(const Scene* scene);

  psl::optional<LightSample> sample(vec3 p, float u1, vec2 u2) const;
  float pdf(vec3 p, const SurfaceInteraction& git, const Ray& ray) const;

private:
  psl::vector<Light> lights;
};

struct LightSampler : private psl::variant<UniformLightSampler> {
  using variant::variant;

  void build(const Scene* scene) {
    return dispatch([&](auto&& x) { return x.build(scene); });
  }
  psl::optional<LightSample> sample(vec3 p, Sampler& sampler) const {
    return dispatch([&](auto&& x) { return x.sample(p, sampler.get1d(), sampler.get2d()); });
  }
  float pdf(const Ray& ray, const SurfaceInteraction& git) const {
    return dispatch([&](auto&& x) { return x.pdf(ray.o, git, ray); });
  }
};

}  // namespace pine
