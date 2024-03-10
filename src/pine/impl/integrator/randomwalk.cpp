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

  for (int depth = 0; depth < max_path_length; depth++)
    if (intersect_cases(
            ray, sampler,
            [&]() {  // Escaped ray
              if (scene.env_light)
                L += beta * scene.env_light->color(ray.d);
              return true;
            },
            [&](const SurfaceInteraction& it) {  // Ray that hits surface
              if (it.material()->is<EmissiveMaterial>()) {
                L += beta * it.material()->le({it, -ray.d});
                return true;
              }
              if (depth + 1 == max_path_length)
                return true;
              auto bs = it.material()->sample({it, -ray.d, sampler.get1d(), sampler.get2d()});
              if (!bs)
                return true;
              beta *= absdot(bs->wo, it.n) * bs->f / bs->pdf;
              ray = it.spawn_ray(bs->wo);
              return false;
            },
            [&](const MediumInteraction& it) {  // Ray that hits paricipating medium
              if (depth + 1 == max_path_length)
                return true;
              ray = Ray(ray(it.t), it.pg.sample(-ray.d, sampler.get2d()).wo);
              return false;
            }))
      break;

  return L;
}

}  // namespace pine