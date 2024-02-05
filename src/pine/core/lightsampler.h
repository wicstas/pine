#pragma once

#include <pine/core/light.h>

#include <psl/optional.h>
#include <psl/variant.h>
#include <psl/vector.h>

namespace pine {

struct UniformLightSampler {
  void build(const Scene* scene);

  psl::optional<LightSample> sample(vec3 p, vec3 n, float u1, vec2 u2) const;
  float pdf(const Geometry* light, const Interaction& it, const Ray& ray, vec3 n) const;

private:
  psl::vector<Light> lights;
};

struct LightSampler : private psl::variant<UniformLightSampler> {
  using variant::variant;

  void build(const Scene* scene) {
    return dispatch([&](auto&& x) { return x.build(scene); });
  }
  psl::optional<LightSample> sample(vec3 p, vec3 n, float u1, vec2 u2) const {
    return dispatch([&](auto&& x) { return x.sample(p, n, u1, u2); });
  }
  float pdf(const Geometry* light, const Interaction& it, const Ray& ray, vec3 n) const {
    return dispatch([&](auto&& x) { return x.pdf(light, it, ray, n); });
  }
};

}  // namespace pine
