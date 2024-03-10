#include <pine/core/lightsampler.h>
#include <pine/core/scene.h>

namespace pine {

void UniformLightSampler::build(const Scene* scene) {
  lights = scene->lights;
  if (scene->env_light)
    scene->env_light->dispatch([&](auto&& x) { lights.push_back(x); });
}

psl::optional<LightSample> UniformLightSampler::sample(vec3 p, vec3 n, float u1, vec2 u2) const {
  auto N = lights.size();
  if (N == 0)
    return psl::nullopt;
  auto index = psl::min(size_t(u1 * N), N - 1);
  if (auto s = lights[index].sample(p, n, u2)) {
    if (s->pdf == 0.0f)
      return psl::nullopt;
    s->light = &lights[index];
    s->pdf = s->pdf / N;
    s->is_delta = s->light->is_delta();
    return s;
  } else {
    return psl::nullopt;
  }
}
float UniformLightSampler::pdf(const Geometry* light, const SurfaceInteraction& it, const Ray& ray,
                               vec3 n) const {
  CHECK(lights.size() != 0);
  CHECK(light != nullptr);
  return light->pdf(it, ray, n) / lights.size();
}

}  // namespace pine