#include <pine/impl/integrator/randomwalk.h>
#include <pine/core/scene.h>

namespace pine {

RandomWalkIntegrator::RandomWalkIntegrator(Accel accel, Sampler sampler, int max_path_length)
    : RayIntegrator{psl::move(accel), psl::move(sampler)}, max_path_length{max_path_length} {
  if (max_path_length <= 0)
    Fatal("`RandomWalkIntegrator` expect `max_path_length` to be positive, get", max_path_length);
}

vec3 RandomWalkIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler) {
  auto L = vec3{0.0f};
  auto beta = vec3{1.0f};

  for (int depth = 0; depth < max_path_length; depth++) {
    auto wi = -ray.d;
    auto ittr = intersect_tr(ray, sampler);
    if (!ittr) {
      if (scene.env_light)
        L += beta * scene.env_light->color(ray.d);
      break;
    } else if (ittr->is<MediumInteraction>()) {
      if (depth + 1 >= max_path_length)
        break;
      auto& ms = ittr->as<MediumInteraction>();
      auto ps = ms.pg.sample(wi, sampler.get2d());
      beta *= ps.f / ps.pdf;
      ray = Ray(ms.p, ps.wo);
    } else {
      auto& it = ittr->as<SurfaceInteraction>();
      if (it.material()->is<EmissiveMaterial>()) {
        L += beta * it.material()->le({it, wi});
        break;
      }

      if (depth + 1 >= max_path_length)
        break;

      if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
        auto cosine = absdot(bs->wo, it.n);
        beta *= bs->f * (cosine / bs->pdf);
        ray = it.spawn_ray(bs->wo);
      } else {
        break;
      }
    }
  }
  return L;
}

}  // namespace pine