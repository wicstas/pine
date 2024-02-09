#pragma once

#include <pine/core/ray.h>
#include <pine/core/log.h>

namespace pine {

struct Interaction {
  Ray spawn_ray(vec3 wo, float distance = float_max) const {
    Ray ray;
    ray.d = wo;
    ray.o = offset_ray_origin(p, face_same_hemisphere(n, ray.d));
    ray.tmin = 1e-6f;
    ray.tmax = distance * (1.0f - 1e-3f);
    return ray;
  }

  vec3 p;
  vec3 n;
  vec2 uv;

  const Material* material() const;

  const Geometry* geometry = nullptr;
  float bvh = 0.0f;
};

}  // namespace pine
