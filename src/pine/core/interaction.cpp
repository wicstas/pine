#include <pine/core/interaction.h>
#include <pine/core/geometry.h>

namespace pine {

Ray SurfaceInteraction::spawn_ray(vec3 wo, float distance) const {
  Ray ray;
  ray.d = wo;
  ray.o = offset_ray_origin(p, face_same_hemisphere(n, ray.d));
  ray.tmin = 0.0f;
  ray.tmax = distance * (1.0f - 1e-3f);
  return ray;
}

const Material& SurfaceInteraction::material() const {
  DCHECK(geometry);
  DCHECK(geometry->material.get());
  return *geometry->material;
}

}  // namespace pine
