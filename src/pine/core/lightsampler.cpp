#include <pine/core/lightsampler.h>
#include <pine/core/scene.h>

namespace pine {

void UniformLightSampler::build(const Scene* scene) {
  lights = scene->lights;
  if (scene->env_light)
    scene->env_light->dispatch([&](auto&& x) { lights.push_back(x); });
}

psl::optional<LightSample> UniformLightSampler::sample(const Interaction& it, float u1,
                                                       vec2 u2) const {
  auto N = lights.size();
  if (N == 0)
    return psl::nullopt;
  auto index = psl::min(size_t(u1 * N), N - 1);
  if (auto s = lights[index].sample(it, u2)) {
    s->light = &lights[index];
    s->pdf = s->pdf / N;
    s->is_delta = s->light->is_delta();
    return s;
  } else {
    return psl::nullopt;
  }
}
float UniformLightSampler::pdf(const Interaction& it, const SurfaceInteraction& git,
                               const Ray& ray) const {
  return git.shape->pdf(it, git, ray) / lights.size();
}

}  // namespace pine