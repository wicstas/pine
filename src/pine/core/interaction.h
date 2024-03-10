#pragma once
#include <pine/core/medium.h>
#include <pine/core/ray.h>
#include <pine/core/log.h>

#include <psl/variant.h>

namespace pine {

struct SurfaceInteraction {
  Ray spawn_ray(vec3 wo, float distance = float_max) const {
    Ray ray;
    ray.d = wo;
    ray.o = offset_ray_origin(p, face_same_hemisphere(n, ray.d));
    ray.tmin = 1e-8f;
    ray.tmax = distance * (1.0f - 1e-3f);
    return ray;
  }

  void compute_transformation() {
    world_to_local = inverse(coordinate_system(n));
  }
  vec3 to_local(vec3 w) const {
    return world_to_local * w;
  }

  vec3 p;
  vec3 n;
  vec2 uv;
  mat3 world_to_local;

  const Material* material() const;

  const Geometry* geometry = nullptr;
  float bvh = 0.0f;
};

struct MediumInteraction {
  float t;
  PhaseFunction pg;
};

struct Interaction : psl::variant<SurfaceInteraction, MediumInteraction> {
  using variant::variant;
};

}  // namespace pine
