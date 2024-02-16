#include <pine/impl/integrator/ao.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>
#include <pine/core/parallel.h>
#include <pine/core/fileio.h>

namespace pine {

static const vec3 directions[8]{uniform_sphere({0.03f, 0.257f}), uniform_sphere({0.257f, 0.257f}),
                                uniform_sphere({0.52f, 0.254f}), uniform_sphere({0.752f, 0.252f}),
                                uniform_sphere({0.09f, 0.759f}), uniform_sphere({0.256f, 0.755f}),
                                uniform_sphere({0.52f, 0.754f}), uniform_sphere({0.753f, 0.759f})};

AOIntegrator::AOIntegrator(Accel accel, Sampler sampler)
    : RayIntegrator(psl::move(accel), psl::move(sampler)) {
  samples_per_pixel = psl::max(samples_per_pixel / 8, 1);
}
vec3 AOIntegrator::radiance(Scene&, Ray ray, Interaction it, bool is_hit, Sampler& sampler) {
  if (is_hit) {
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