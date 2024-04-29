#pragma once
#include <pine/core/phase_function.h>
#include <pine/core/ray.h>
#include <pine/core/log.h>

#include <psl/variant.h>

namespace pine {

struct SurfaceInteraction {
  Ray spawn_ray(vec3 wo, float distance = float_max) const;

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

  const Material& material() const;

  const Shape* shape = nullptr;
  const Material* _material = nullptr;
};

struct MediumInteraction {
  MediumInteraction() = default;
  MediumInteraction(float t, vec3 p, vec3 W, PhaseFunction pg)
      : t(t), p(p), W(W), pg(psl::move(pg)) {
  }
  MediumInteraction(float t, vec3 p, vec3 W, vec3 le) : t(t), p(p), W(W), le(le) {
  }

  float t;
  vec3 p;
  vec3 W;
  PhaseFunction pg;
  psl::optional<vec3> le;
};

struct Interaction : psl::variant<SurfaceInteraction, MediumInteraction> {
  using variant::variant;
  vec3 p() const {
    return dispatch([](auto&& x) { return x.p; });
  }
  vec3 surface_n() const {
    return as<SurfaceInteraction>().n;
  }
};

}  // namespace pine
