#include <pine/core/lightsampler.h>
#include <pine/core/scene.h>

namespace pine {

void UniformLightSampler::build(const Scene* scene) {
  lights = scene->lights;
  if (scene->env_light)
    scene->env_light->dispatch([&](auto&& x) { lights.push_back(x); });
}

psl::optional<LightSample> UniformLightSampler::sample(vec3 p, float u1, vec2 u2) const {
  auto N = int(lights.size());
  if (N == 0)
    return psl::nullopt;
  u1 *= N;
  auto index = int(u1);
  if (auto s = lights[index].sample(p, u2, u1 - index)) {
    s->light = &lights[index];
    s->pdf = s->pdf / N;
    s->is_delta = s->light->is_delta();
    return s;
  } else {
    return psl::nullopt;
  }
}
float UniformLightSampler::pdf(vec3, const SurfaceInteraction& git, const Ray& ray) const {
  return git.shape->pdf(ray, git.p, git.n) / lights.size();
}

}  // namespace pine