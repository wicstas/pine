#include <pine/core/interaction.h>
#include <pine/core/sampling.h>
#include <pine/core/bbox.h>
#include <pine/core/log.h>

namespace pine {

AABB::AABB(OBB obb) {
  for (int i = 0; i < 8; i++) {
    auto p = obb.base.lower;
    if (i % 2 >= 1) p[0] = obb.base.upper[0];
    if (i % 4 >= 2) p[1] = obb.base.upper[1];
    if (i % 8 >= 4) p[2] = obb.base.upper[2];
    extend(obb.m * p);
  }
}
AABB AABB::extend_to_max_axis() const {
  auto c = centroid();
  auto d = vec3(max_value(diagonal()) / 2);
  return {c - d, c + d};
}
vec3 AABB::relative_position(vec3 p) const {
  vec3 o = p - lower;
  if (upper.x > lower.x) o.x /= upper.x - lower.x;
  if (upper.y > lower.y) o.y /= upper.y - lower.y;
  if (upper.z > lower.z) o.z /= upper.z - lower.z;
  return o;
}
vec3 AABB::absolute_position(vec3 p) const {
  return {lerp(p.x, lower.x, upper.x), lerp(p.y, lower.y, upper.y), lerp(p.z, lower.z, upper.z)};
}
float AABB::relative_position(float p, int dim) const {
  float o = p - lower[dim];
  float d = upper[dim] - lower[dim];
  return d > 0.0f ? o / d : o;
}
float AABB::surface_area() const {
  vec3 d = diagonal();
  return 2.0f * (d.x * d.y + d.x * d.z + d.y * d.z);
}
AABB& AABB::extend(vec3 p) {
  lower = min(lower, p);
  upper = max(upper, p);
  return *this;
}
AABB& AABB::extend(const AABB& aabb) {
  lower = min(lower, aabb.lower);
  upper = max(upper, aabb.upper);
  return *this;
}
AABB& AABB::extend_by(float amount) {
  lower -= vec3(amount);
  upper += vec3(amount);
  return *this;
}
psl::pair<AABB, AABB> AABB::split_half(int axis) const {
  CHECK_RANGE(axis, 0, 2);
  auto left = *this;
  auto right = *this;
  left.upper[axis] = centroid(axis);
  right.lower[axis] = centroid(axis);
  return {left, right};
}
bool AABB::contains(vec3 p) const {
  return (p[0] >= lower[0] && p[0] <= upper[0]) && (p[1] >= lower[1] && p[1] <= upper[1]) &&
         (p[2] >= lower[2] && p[2] <= upper[2]);
}
bool AABB::hit(const Ray& ray) const {
  float tmin = ray.tmin;
  float tmax = ray.tmax;
  if (tmin > tmax) return false;
  for (int i = 0; i < 3; i++) {
    if (psl::abs(ray.d[i]) < 1e-6f) {
      if (ray.o[i] < lower[i] || ray.o[i] > upper[i])
        return false;
      else
        continue;
    }
    float inv_d = 1.0f / ray.d[i];
    float t_near = (lower[i] - ray.o[i]) * inv_d;
    float t_far = (upper[i] - ray.o[i]) * inv_d;
    if (inv_d < 0.0f) psl::swap(t_far, t_near);
    tmin = psl::max(t_near, tmin);
    tmax = psl::min(t_far, tmax);
    if (tmin > tmax) return false;
  }
  return true;
}
bool AABB::intersect(vec3 o, vec3 d, float& tmin, float& tmax) const {
  for (int i = 0; i < 3; i++) {
    if (psl::abs(d[i]) < 1e-6f) {
      if (o[i] < lower[i] || o[i] > upper[i])
        return false;
      else
        continue;
    }
    float inv_d = 1.0f / d[i];
    float t_near = (lower[i] - o[i]) * inv_d;
    float t_far = (upper[i] - o[i]) * inv_d;
    if (inv_d < 0.0f) psl::swap(t_far, t_near);
    tmin = psl::max(t_near, tmin);
    tmax = psl::min(t_far, tmax);
    if (tmin > tmax) return false;
  }
  return true;
}
bool AABB::intersect(Ray& ray, SurfaceInteraction&) const {
  auto tmin = ray.tmin;
  auto tmax = ray.tmax;
  if (intersect(ray.o, ray.d, tmin, tmax)) {
    ray.tmax = tmin > ray.tmin ? tmin : tmax;
    return true;
  } else {
    return false;
  }
}
void AABB::compute_surface_info(vec3 p, SurfaceInteraction& it) const {
  it.p = p;
  auto pu = (p - centroid()) / diagonal();
  auto axis = max_axis(abs(pu));
  it.n = vec3(0.0f);
  it.n[axis] = pu[axis] > 0 ? 1 : -1;
  it.p[axis] = pu[axis] > 0 ? upper[axis] : lower[axis];
}
psl::optional<ShapeSample> AABB::sample(vec3, vec2 u) const {
  auto axis = int(u.x * 3);
  u.x = u.x * 3 - axis;
  auto b = with_prob(0.5f, u.y);
  auto p0 = lower;
  if (b) p0[axis] = upper[axis];
  p0[(axis + 1) % 3] += u.x * diagonal((axis + 1) % 3);
  p0[(axis + 2) % 3] += u.y * diagonal((axis + 2) % 3);
  auto ss = ShapeSample();
  ss.p = p0;
  ss.n[axis] = b ? 1 : -1;
  return ss;
}
float AABB::area() const {
  auto d = diagonal();
  return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
}

OBB::OBB(AABB aabb, mat4 m) : base(aabb), m(m), m_inv(inverse(m)) {}
bool OBB::hit(Ray ray) const {
  ray.o = m_inv * ray.o;
  ray.d = normalize(mat3(m_inv) * ray.d);
  return base.hit(ray);
}
bool OBB::intersect(vec3 o, vec3 d, float& tmin, float& tmax) const {
  auto org = o;
  o = m_inv * o;
  d = normalize(mat3(m_inv) * d);
  if (base.intersect(o, d, tmin, tmax)) {
    auto ps = o + tmin * d;
    auto pe = o + tmax * d;
    tmin = distance(m * ps, org);
    tmax = distance(m * pe, org);
    return true;
  } else {
    return false;
  }
}
bool OBB::intersect(Ray& ray, SurfaceInteraction&) const {
  auto tmin = ray.tmin, tmax = ray.tmax;
  if (intersect(ray.o, ray.d, tmin, tmax)) {
    ray.tmax = tmin > ray.tmin ? tmin : tmax;
    return true;
  } else {
    return false;
  }
}
void OBB::compute_surface_info(vec3 p, SurfaceInteraction& it) const {
  base.compute_surface_info(m_inv * p, it);
  it.p = m * it.p;
  it.n = normalize(transpose(mat3(m_inv)) * it.n);
}
psl::optional<ShapeSample> OBB::sample(vec3 p, vec2 u) const {
  auto ss = base.sample(p, u);
  ss->p = m * ss->p;
  return ss;
}
float OBB::area() const { return base.area(); }

}  // namespace pine
