#include <pine/core/aabb.h>
#include <pine/core/log.h>

namespace pine {

AABB AABB::extend_to_max_axis() const {
  auto c = centroid();
  auto d = vec3(max_value(diagonal()) / 2);
  return {c - d, c + d};
}
vec3 AABB::relative_position(vec3 p) const {
  vec3 o = p - lower;
  if (upper.x > lower.x)
    o.x /= upper.x - lower.x;
  if (upper.y > lower.y)
    o.y /= upper.y - lower.y;
  if (upper.z > lower.z)
    o.z /= upper.z - lower.z;
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
void AABB::extend(vec3 p) {
  lower = min(lower, p);
  upper = max(upper, p);
}
void AABB::extend(const AABB& aabb) {
  lower = min(lower, aabb.lower);
  upper = max(upper, aabb.upper);
}
void AABB::extend_by(float amount) {
  lower -= vec3(amount);
  upper += vec3(amount);
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
  if (tmin > tmax)
    return false;
  for (int i = 0; i < 3; i++) {
    float invRayDir = 1.0f / ray.d[i];
    float tNear = (lower[i] - ray.o[i]) * invRayDir;
    float tFar = (upper[i] - ray.o[i]) * invRayDir;
    if (ray.d[i] < 0.0f) {
      float temp = tNear;
      tNear = tFar;
      tFar = temp;
    }
    tmin = tNear > tmin ? tNear : tmin;
    tmax = tFar < tmax ? tFar : tmax;
    if (tmin > tmax)
      return false;
  }
  return true;
}
bool AABB::hit(Ray ray, float& tmin, float& tmax) const {
  tmin = ray.tmin;
  tmax = ray.tmax;
  if (tmin > tmax)
    return false;
  for (int i = 0; i < 3; i++) {
    float invRayDir = 1.0f / ray.d[i];
    float tNear = (lower[i] - ray.o[i]) * invRayDir;
    float tFar = (upper[i] - ray.o[i]) * invRayDir;
    if (ray.d[i] < 0.0f) {
      float temp = tNear;
      tNear = tFar;
      tFar = temp;
    }
    tmin = tNear > tmin ? tNear : tmin;
    tmax = tFar < tmax ? tFar : tmax;
    if (tmin > tmax)
      return false;
  }
  return true;
}

}  // namespace pine
