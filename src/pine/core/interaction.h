#pragma once
#include <pine/core/log.h>
#include <pine/core/phase_function.h>
#include <pine/core/ray.h>
#include <psl/variant.h>

namespace pine {

struct SurfaceInteraction {
  SurfaceInteraction() = default;
  SurfaceInteraction(vec3 p, vec3 n, vec3 uv) : p(p), n(n), uv(uv) { compute_transformation(); }
  Ray spawn_ray(vec3 wo, float tmax = float_max) const;

  void compute_transformation() {
    l2w = coordinate_system(n);
    w2l = transpose(l2w);
  }
  vec3 to_world(vec3 w) const { return l2w * w; }
  vec3 to_local(vec3 w) const { return w2l * w; }
  bool is_inward(const Ray& ray) const { return dot(ray.d, n) < 0; }
  bool is_outward(const Ray& ray) const { return !is_inward(ray); }

  vec3 p;
  vec3 n;
  vec2 uv;
  mat3 w2l, l2w;

  const Material& material() const;

  const Shape* shape = nullptr;
  const Material* _material = nullptr;
};

// struct MediumSample {
//   MediumSample() = default;
//   MediumSample(vec3 p, vec3 W, float pdf, PhaseFunction pg) : p(p), W(W), pg(MOVE(pg)) {
//   }
//   Ray spawn_ray(vec3 wo, float tmax = float_max) const {
//     return Ray(p, wo, 0.0f, tmax * 0.999f);
//   }

//   vec3 p;
//   vec3 W;
//   PhaseFunction pg;
// };

struct Interaction : SurfaceInteraction {
  Interaction(SurfaceInteraction x) : SurfaceInteraction(x) {}
  using SurfaceInteraction::SurfaceInteraction;
};
// struct Interaction : psl::variant<SurfaceInteraction, MediumSample> {
//   using variant::variant;
//   vec3 p() const {
//     return dispatch([](auto&& x) { return x.p; });
//   }
//   vec3 surface_n() const {
//     return as<SurfaceInteraction>().n;
//   }
//   Ray spawn_ray(vec3 wo, float tmax = float_max) const {
//     return dispatch([&](auto&& x) { return x.spawn_ray(wo, tmax); });
//   }
// };

}  // namespace pine
