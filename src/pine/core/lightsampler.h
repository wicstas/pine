#pragma once
#include <pine/core/light.h>

#include <psl/optional.h>
#include <psl/variant.h>
#include <psl/vector.h>

namespace pine {

struct UniformLightSampler {
  void build(const Scene* scene);

  psl::optional<LightSample> sample(const Interaction& it, float u1, vec2 u2) const;
  float pdf(const Interaction& it, const SurfaceInteraction& git, const Ray& ray) const;

private:
  psl::vector<Light> lights;
};

struct LightSampler : private psl::variant<UniformLightSampler> {
  using variant::variant;

  void build(const Scene* scene) {
    return dispatch([&](auto&& x) { return x.build(scene); });
  }
  psl::optional<LightSample> sample(const Interaction& it, float u1, vec2 u2) const {
    return dispatch([&](auto&& x) { return x.sample(it, u1, u2); });
  }
  float pdf(const Interaction& it, const SurfaceInteraction& git, const Ray& ray) const {
    return dispatch([&](auto&& x) { return x.pdf(it, git, ray); });
  }
};

}  // namespace pine
