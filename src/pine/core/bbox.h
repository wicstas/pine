#pragma once
#include <pine/core/ray.h>
#include <pine/core/log.h>

namespace pine {

struct ShapeSample {
  vec3 p;
  vec3 n;
  vec2 uv;
  vec3 w;
  float distance = 0.0f;
  float pdf = 0.0f;
};

struct RayOctant {
  RayOctant(const Ray& ray)
      : octantx3{psl::signbit(ray.d[0]) * 3, psl::signbit(ray.d[1]) * 3,
                 psl::signbit(ray.d[2]) * 3},
        dir_inv(safe_rcp(ray.d)),
        org_div_dir(ray.o * dir_inv) {
  }

  int octantx3[3];
  vec3 dir_inv, org_div_dir;
};

struct AABB {
  AABB() = default;
  AABB(vec3 lower, vec3 upper) : lower(lower), upper(upper){};
  AABB(vec3 p) : lower(p), upper(p){};
  explicit AABB(struct OBB);

  vec3 centroid() const {
    return (lower + upper) / 2;
  }
  float centroid(int dim) const {
    return (lower[dim] + upper[dim]) / 2;
  }
  float diagonal(int dim) const {
    return upper[dim] - lower[dim];
  }
  vec3 diagonal() const {
    return upper - lower;
  }
  AABB extend_to_max_axis() const;
  vec3 relative_position(vec3 p) const;
  vec3 absolute_position(vec3 p) const;
  float relative_position(float p, int dim) const;
  float surface_area() const;
  void extend(vec3 p);
  void extend(const AABB& aabb);
  void extend_by(float amount);
  psl::pair<AABB, AABB> split_half(int axis) const;

  friend AABB union_(AABB l, AABB r) {
    l.extend(r);
    return l;
  }
  bool degenerated() const {
    return degenerated(0) || degenerated(1) || degenerated(2);
  }
  bool degenerated(int dim) const {
    return upper[dim] <= lower[dim];
  }
  bool contains(vec3 p) const;

  bool hit(const Ray& ray) const;
  bool intersect(vec3 o, vec3 d, float& tmin, float& tmax) const;

  PINE_ALWAYS_INLINE bool hit(const RayOctant& r, float tmin, float& tmax) const {
    auto p = &lower[0];
    auto tmin0 = p[0 + r.octantx3[0]] * r.dir_inv[0] - r.org_div_dir[0];
    auto tmin1 = p[1 + r.octantx3[1]] * r.dir_inv[1] - r.org_div_dir[1];
    auto tmin2 = p[2 + r.octantx3[2]] * r.dir_inv[2] - r.org_div_dir[2];

    auto tmax0 = p[3 - r.octantx3[0]] * r.dir_inv[0] - r.org_div_dir[0];
    auto tmax1 = p[4 - r.octantx3[1]] * r.dir_inv[1] - r.org_div_dir[1];
    auto tmax2 = p[5 - r.octantx3[2]] * r.dir_inv[2] - r.org_div_dir[2];

    tmin = psl::max(tmin0, tmin1, tmin2, tmin);
    tmax = psl::min(tmax0, tmax1, tmax2, tmax);
    return tmin <= tmax;
  }
  psl::string to_string() const {
    return psl::to_string("{", lower, ", ", upper, "}");
  }

  bool intersect(Ray& ray) const;
  void compute_surface_info(SurfaceInteraction&) const;
  AABB get_aabb() const {
    return *this;
  }
  psl::optional<ShapeSample> sample(vec3, vec2) const {
    PINE_UNREACHABLE;
  }
  float pdf(const Interaction&, const SurfaceInteraction&, const Ray&) const {
    PINE_UNREACHABLE;
  }
  float area() const {
    PINE_UNREACHABLE;
  }

  vec3 lower = vec3{float_max};
  vec3 upper = vec3{-float_max};
};

struct OBB {
  OBB() = default;
  OBB(AABB aabb, mat4 m);
  bool hit(Ray ray) const;
  bool intersect(vec3 o, vec3 d, float& tmin, float& tmax) const;
  bool intersect(Ray& ray) const;
  void compute_surface_info(SurfaceInteraction&) const {
    PINE_UNREACHABLE;
  }
  AABB get_aabb() const {
    return AABB(*this);
  }
  psl::optional<ShapeSample> sample(vec3, vec2) const {
    PINE_UNREACHABLE;
  }
  float pdf(const Interaction&, const SurfaceInteraction&, const Ray&) const {
    PINE_UNREACHABLE;
  }
  float area() const {
    PINE_UNREACHABLE;
  }

  vec3 p;
  vec3 dim;
  mat3 n;
};

}  // namespace pine
