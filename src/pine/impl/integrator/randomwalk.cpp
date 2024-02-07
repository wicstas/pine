#include <pine/impl/integrator/randomwalk.h>
#include <pine/core/scene.h>

namespace pine {

vec3 RandomWalkIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler) {
  auto L = vec3{0.0f};
  auto beta = vec3{1.0f};

  for (int depth = 0; depth < max_depth; depth++) {
    auto it = Interaction{};
    if (!intersect(ray, it)) {
      if (scene.env_light) {
        L += beta * scene.env_light->color(ray.d);
      }
      break;
    }
    if (it.geometry->material.get()->is<EmissiveMaterial>()) {
      L += beta * it.geometry->material.get()->le({it, -ray.d});
      break;
    }

    if (depth + 1 == max_depth)
      break;

    if (auto bs = it.geometry->material.get()->sample({it, -ray.d, sampler.get1d(), sampler.get2d()})) {
      beta *= absdot(bs->wo, it.n) * bs->f / bs->pdf;
      if (beta.is_black())
        break;
      ray = it.spawn_ray(bs->wo);
    } else {
      break;
    }
  }

  return L;
}

}  // namespace pine