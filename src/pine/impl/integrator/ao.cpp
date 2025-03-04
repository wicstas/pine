#include <pine/impl/integrator/ao.h>
#include <pine/core/scene.h>

namespace pine {

static const vec3 directions[8]{uniform_sphere({0.0f, 0.25f}), uniform_sphere({0.25f, 0.25f}),
                                uniform_sphere({0.5f, 0.25f}), uniform_sphere({0.75f, 0.25f}),
                                uniform_sphere({0.0f, 0.75f}), uniform_sphere({0.25f, 0.75f}),
                                uniform_sphere({0.5f, 0.75f}), uniform_sphere({0.75f, 0.75f})};

AOIntegrator::AOIntegrator(Accel accel, Sampler sampler)
    : RayIntegrator{MOVE(accel), MOVE(sampler), UniformLightSampler()} {
  spp = psl::max(spp / 8, 1);
}
vec3 AOIntegrator::radiance(Scene&, Ray ray, Sampler& sampler) {
  auto it = SurfaceInteraction();
  if (intersect(ray, it)) {
    it.n = face_same_hemisphere(it.n, -ray.d);
    Ray rays[8];
    auto tbn = coordinate_system(uniform_sphere(sampler.get2d()));
    for (int i = 0; i < 8; i++)
      rays[i] = it.spawn_ray(face_same_hemisphere(tbn * directions[i], it.n), radius);
    auto occluded = hit8(rays);
    auto ao = vec3(0.0f);
    for (int i = 0; i < 8; i++)
      if (((occluded >> i) & 1) == 0)
        ao += vec3(1.0f / 8.0f);
    return ao;
  }
  return vec3(0.0f);
}

}  // namespace pine