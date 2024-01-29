#include <pine/impl/integrator/ao.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>
#include <pine/core/parallel.h>
#include <pine/core/fileio.h>

namespace pine {

vec3 AOIntegrator::radiance(Scene&, Ray ray, Sampler& sampler) {
  auto it = Interaction{};
  if (intersect(ray, it)) {
    ray.o = it.p;
    it.n = face_same_hemisphere(it.n, -ray.d);
    ray = it.spawn_ray(face_same_hemisphere(uniform_hemisphere(sampler.get2d()), it.n));
    return hit(ray) ? vec3(0.0f) : vec3(1.0f);
  }
  return vec3(0.0f);
}

}  // namespace pine