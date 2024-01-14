#include <pine/core/lightsampler.h>
#include <pine/core/scene.h>

namespace pine {

void UniformLightSampler::build(const Scene* scene) {
  lights = scene->lights;
}

psl::optional<LightSample> UniformLightSampler::sample(vec3 p, vec3, float u1, vec2 u2) const {
  auto N = lights.size();
  if (N == 0)
    return psl::nullopt;
  auto index = psl::min(static_cast<size_t>(u1 * N), N - 1);
  auto s = lights[index].sample(p, u2);
  if (s.pdf == 0.0f)
    return psl::nullopt;
  s.light = &lights[index];

  s.pdf = s.pdf / N;

  return s;
}
float UniformLightSampler::pdf(const Geometry* light, const Interaction& it, const Ray& ray,
                               vec3 n) const {
  CHECK(lights.size() != 0);
  CHECK(light != nullptr);
  return light->pdf(it, ray, n) / lights.size();
}

}  // namespace pine