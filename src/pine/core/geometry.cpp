#include <pine/core/accel.h>
#include <pine/core/context.h>
#include <pine/core/fileio.h>
#include <pine/core/geometry.h>
#include <pine/core/parallel.h>

#include <mutex>

namespace pine {

bool hit_quadratic(float a, float b, float c, float tmin, float tmax) {
  float d = b * b - 4 * a * c;
  if (d <= 0.0f) return false;
  d = psl::sqrt(d);
  float t = (-b - d) / (2 * a);
  if (t < tmin) t += d / a;
  if (t < tmin || t > tmax) return false;
  return true;
}
bool intersect_quadratic(float a, float b, float c, float tmin, float& tmax) {
  float d = b * b - 4 * a * c;
  if (d <= 0.0f) return false;
  d = psl::sqrt(d);
  float t = (-b - d) / (2 * a);
  if (t < tmin) t += d / a;
  if (t < tmin || t > tmax) return false;
  tmax = t;
  return true;
}

Plane::Plane(vec3 position, vec3 normal) : position(position), n(normalize(normal)) {
  if (length(n) == 0) Fatal("`Plane` can't have degenerated normal");
  coordinate_system(n, u, v);
}
bool Plane::hit(const Ray& ray) const {
  float t = (dot(position, n) - dot(ray.o, n)) / dot(ray.d, n);
  if (t <= ray.tmin) return false;
  return t < ray.tmax;
}
bool Plane::intersect(Ray& ray, SurfaceInteraction&) const {
  float t = (dot(position, n) - dot(ray.o, n)) / dot(ray.d, n);
  if (t < ray.tmin || t > ray.tmax) return false;
  ray.tmax = t;
  return true;
}
void Plane::compute_surface_info(vec3 p, SurfaceInteraction& it) const {
  it.n = n;
  auto dp = p - position;
  it.uv = vec2(dot(dp, u), dot(dp, v));
  it.p = position + it.uv.x * u + it.uv.y * v;
}
AABB Plane::get_aabb() const { return {position + vec3(-100.0f), position + vec3(100.0f)}; }
static vec3 project_to_plane(vec3 plane_p, vec3 plane_ex, vec3 plane_ey, vec3 p) {
  auto dp = p - plane_p;
  return plane_p + plane_ex * dot(plane_ex, dp) + plane_ey * dot(plane_ey, dp);
}
psl::optional<ShapeSample> Plane::sample(vec3 p, vec2 u2) const {
  auto ss = ShapeSample{};
  auto p_sphere = uniform_hemisphere(u2);
  auto l = absdot(p - position, n);
  auto ex = l * p_sphere.x / p_sphere.z;
  auto ey = l * p_sphere.y / p_sphere.z;
  ss.p = project_to_plane(position, u, v, p) + u * ex + v * ey;
  ss.n = n;
  ss.uv = {ex, ey};
  ss.w = normalize(ss.p - p, ss.distance);
  ss.pdf = 1.0f / (2 * Pi);
  return ss;
}
float Plane::pdf(const Ray&, vec3, vec3) const { return 1.0f / (2 * Pi); }

Sphere::Sphere(vec3 position, float radius) : c(position), r(radius) {}
float Sphere::compute_t(vec3 ro, vec3 rd, float tmin, vec3 p, float r) {
  auto ro_p = ro - p;
  float b = dot(ro_p, rd);
  float c = dot(ro_p, ro_p) - r * r;
  float d = b * b - c;
  if (d <= 0.0f) return -1.0f;
  d = psl::sqrt(d);
  float t = -b - d;
  if (t < tmin) t = -b + d;
  return t;
}
bool Sphere::hit(const Ray& ray) const {
  float t = compute_t(ray.o, ray.d, ray.tmin, c, r);
  return t > ray.tmin && t < ray.tmax;
}
bool Sphere::intersect(Ray& ray, SurfaceInteraction&) const {
  float t = compute_t(ray.o, ray.d, ray.tmin, c, r);
  if (t < ray.tmin || t > ray.tmax) return false;
  ray.tmax = t;
  return true;
}
void Sphere::compute_surface_info(vec3 p, SurfaceInteraction& it) const {
  it.n = normalize(p - this->c);
  it.p = c + it.n * r;
  it.uv = cartesian_to_spherical(it.n);
}
psl::optional<ShapeSample> Sphere::sample(vec3 p, vec2 u) const {
  auto ss = ShapeSample{};
  auto l = length(c - p);
  auto cos_theta = psl::sqrt(1 - psl::sqr(r / l));
  auto S = 2 * Pi * (1 - cos_theta);
  auto cos_theta_wo = 1 - u.y * (1 - cos_theta);
  auto sin_theta_wo = psl::sqrt(1 - cos_theta_wo * cos_theta_wo);
  ss.w = spherical_to_cartesian(u.x * 2 * Pi, sin_theta_wo, cos_theta_wo);
  ss.w = coordinate_system((c - p) / l) * ss.w;
  ss.distance = compute_t(p, ss.w, 0.0f, c, r);
  ss.pdf = 1.0f / S;
  ss.p = p + ss.w * ss.distance;
  ss.n = (ss.p - c) / r;
  ss.uv = cartesian_to_spherical(ss.n);
  return ss;
}
float Sphere::pdf(const Ray& ray, vec3, vec3) const {
  auto l = length(c - ray.o);
  auto cos_theta = psl::sqrt(1 - psl::sqr(r / l));
  auto S = 2 * Pi * (1 - cos_theta);
  return 1.0f / S;
}
AABB Sphere::get_aabb() const { return {c - vec3(r), c + vec3(r)}; }

Disk::Disk(vec3 position, vec3 normal, float r) : position(position), n(normalize(normal)), r(r) {
  coordinate_system(n, u, v);
  if (r < 0.0f) Fatal("`Disk` can't have negative radius ", r);
  if (length(n) == 0.0f) Fatal("`Disk` can't have degenerated normal");
}
bool Disk::hit(const Ray& ray) const {
  auto denom = dot(ray.d, n);
  if (denom == 0.0f) return false;
  float t = (dot(position, n) - dot(ray.o, n)) / denom;
  if (t < ray.tmin) return false;
  if (t >= ray.tmax) return false;
  vec3 p = ray(t) - position;
  if (length_squared(p) > psl::sqr(r)) return false;
  return true;
}
bool Disk::intersect(Ray& ray, SurfaceInteraction&) const {
  auto denom = dot(ray.d, n);
  if (denom == 0.0f) return false;
  float t = (dot(position, n) - dot(ray.o, n)) / denom;
  if (t < ray.tmin || t > ray.tmax) return false;
  vec3 p = ray(t) - position;
  if (length_squared(p) > psl::sqr(r)) return false;

  ray.tmax = t;
  return true;
}
void Disk::compute_surface_info(vec3 p, SurfaceInteraction& it) const {
  it.n = n;
  auto ex = dot(p - position, u);
  auto ey = dot(p - position, v);
  it.uv = {ex, ey};
  it.p = position + ex * u + ey * v;
}
psl::optional<ShapeSample> Disk::sample(vec3 p, vec2 u2) const {
  auto ss = ShapeSample{};
  auto uv = sample_disk_concentric(u2);
  ss.p = position + r * u * uv[0] + r * v * uv[1];
  ss.n = n;
  ss.uv = u2;
  ss.w = normalize(ss.p - p, ss.distance);
  ss.pdf = psl::sqr(ss.distance) / psl::max(absdot(ss.w, ss.n) * area(), epsilon);
  return ss;
}
float Disk::pdf(const Ray& ray, vec3, vec3 ns) const {
  return psl::sqr(ray.tmax) / (area() * absdot(ns, ray.d));
}
AABB Disk::get_aabb() const { return Sphere(position, r).get_aabb(); }

Line::Line(vec3 p0, vec3 p1, float thickness)
    : p0(p0),
      p1(p1),
      tbn(coordinate_system(normalize(p1 - p0))),
      thickness(thickness),
      len{length(p1 - p0)} {
  if (thickness <= 0.0f) Fatal("`Line` should have positive thickness, not ", thickness);
  if (p0 == p1) Fatal("`Line` shouldn't have identical begin and end point ", p0);
}
bool Line::hit(const Ray& ray) const {
  auto r2o = look_at(ray.o, ray.o + ray.d);
  auto o2r = inverse(r2o);
  auto p0 = vec3{o2r * vec4(this->p0, 1.0f)};
  auto p1 = vec3{o2r * vec4(this->p1, 1.0f)};

  auto o = p0;
  auto d = p1 - p0;
  auto tz = inverse(mat2(dot(d, d), -d.z, -d.z, 1.0f)) * vec2(-dot(o, d), o.z);
  auto t = psl::clamp(tz.x, 0.0f, 1.0f);
  auto z = psl::clamp(o.z + t * d.z, ray.tmin + thickness, ray.tmax);

  auto D = length(o + t * d - vec3(0.0f, 0.0f, z));

  return D <= thickness;
}
bool Line::intersect(Ray& ray, SurfaceInteraction&) const {
  auto r2o = look_at(ray.o, ray.o + ray.d);
  auto o2r = inverse(r2o);
  auto p0 = vec3(o2r * vec4(this->p0, 1.0f));
  auto p1 = vec3(o2r * vec4(this->p1, 1.0f));

  auto o = p0;
  auto d = p1 - p0;
  auto tz = inverse(mat2(dot(d, d), -d.z, -d.z, 1.0f)) * vec2(-dot(o, d), o.z);
  auto t = psl::clamp(tz.x, 0.0f, 1.0f);
  auto z = psl::clamp(o.z + t * d.z, ray.tmin + thickness, ray.tmax);

  auto D = length(o + t * d - vec3(0.0f, 0.0f, z));

  if (D > thickness) return false;

  ray.tmax = z;
  return true;
}
void Line::compute_surface_info(vec3 p, SurfaceInteraction& it) const {
  auto lt = dot(p - this->p0, tbn.z);
  auto lp = lerp(lt, this->p0, this->p1);
  it.p = p;
  it.n = normalize(p - lp);
  it.uv = {lt, 0.0f};
  // TODO
}
psl::optional<ShapeSample> Line::sample(vec3 p, vec2 u2) const {
  auto ss = ShapeSample{};
  auto phi = u2[1] * 2 * Pi;
  ss.p =
      lerp(u2[0], p0, p1) + thickness * psl::cos(phi) * tbn.x + thickness * psl::sin(phi) * tbn.y;
  ss.n = psl::cos(phi) * tbn.x + psl::sin(phi) * tbn.y;
  ss.uv = u2;
  ss.w = normalize(ss.p - p, ss.distance);
  ss.pdf = psl::sqr(ss.distance) / (absdot(ss.w, ss.n) * area());
  return ss;
}
float Line::pdf(const Ray& ray, vec3, vec3 ns) const {
  return psl::sqr(ray.tmax) / (area() * absdot(ns, ray.d));
}
AABB Line::get_aabb() const {
  AABB aabb;
  aabb.extend(p0 - vec3(thickness));
  aabb.extend(p1 - vec3(thickness));
  aabb.extend(p0 + vec3(thickness));
  aabb.extend(p1 + vec3(thickness));
  return aabb;
}

struct FuzzyValue {
  friend bool operator==(FuzzyValue a, float b) { return psl::abs(a.value - b) <= 1e-6f; }
  friend bool operator==(float a, FuzzyValue b) { return psl::abs(a - b.value) <= 1e-6f; }
  friend bool operator!=(FuzzyValue a, float b) { return psl::abs(a.value - b) > 1e-6f; }
  friend bool operator!=(float a, FuzzyValue b) { return psl::abs(a - b.value) > 1e-6f; }
  float value;
};
auto roughly(float x) { return FuzzyValue{x}; }

Rect::Rect(vec3 position, vec3 ex, vec3 ey, bool flip_normal)
    : position(position),
      ex(normalize(ex)),
      ey(normalize(ey)),
      n(normalize(cross(this->ex, this->ey)) * (flip_normal ? -1 : 1)),
      lx(length(ex)),
      ly(length(ey)),
      rx(this->ex / lx),
      ry(this->ey / ly) {
  if (length(n) != roughly(1)) Fatal("`Rect` has degenerated shape");
}
Rect Rect::from_vertex(vec3 v0, vec3 v1, vec3 v2) {
  auto ex = v1 - v0;
  auto ey = v2 - v0;
  return Rect{v0 + ex / 2 + ey / 2, ex, ey};
}
Rect& Rect::apply(mat4 m) {
  auto v0 = position - ex * lx / 2 - ey * ly / 2;
  auto v1 = v0 + ex * lx;
  auto v2 = v0 + ey * ly;
  v0 = vec3{m * vec4{v0, 1.0f}};
  v1 = vec3{m * vec4{v1, 1.0f}};
  v2 = vec3{m * vec4{v2, 1.0f}};
  return *this = from_vertex(v0, v1, v2);
}
bool Rect::hit(const Ray& ray) const {
  float denom = dot(ray.d, n);
  if (denom == 0.0f) return false;
  float t = (dot(position - ray.o, n)) / denom;
  if (t <= ray.tmin || t >= ray.tmax) return false;
  vec3 p = ray(t) - position;
  float u = dot(p, rx);
  if (u < -0.5f || u > 0.5f) return false;
  float v = dot(p, ry);
  if (v < -0.5f || v > 0.5f) return false;
  return true;
}
bool Rect::intersect(Ray& ray, SurfaceInteraction&) const {
  float denom = dot(ray.d, n);
  if (denom == 0.0f) return false;
  float t = (dot(position - ray.o, n)) / denom;
  if (t <= ray.tmin || t >= ray.tmax) return false;
  vec3 p = ray(t) - position;
  float u = dot(p, rx);
  if (u < -0.5f || u > 0.5f) return false;
  float v = dot(p, ry);
  if (v < -0.5f || v > 0.5f) return false;
  ray.tmax = t;
  return true;
}
void Rect::compute_surface_info(vec3 p, SurfaceInteraction& it) const {
  auto rp = p - position;
  float u = dot(rp, rx);
  float v = dot(rp, ry);
  it.p = position + lx * ex * u + ly * ey * v;
  it.n = n;
  it.uv = vec2(u, v) + vec2(0.5f);
}
psl::optional<ShapeSample> Rect::sample(vec3 o, vec2 u) const {
  auto ss = ShapeSample{};
  ss.p = position + (u[0] - 0.5f) * ex * lx + (u[1] - 0.5f) * ey * ly;
  ss.n = n;
  ss.uv = u;
  ss.w = normalize(ss.p - o, ss.distance);
  ss.pdf = psl::sqr(ss.distance) / (absdot(ss.w, ss.n) * area());
  return ss;

  //   auto p = position - ex * lx / 2 - ey * ly / 2;
  //   auto ez = cross(ex, ey);
  //   auto dr = p - o;
  //   auto z0 = dot(dr, ez);
  //   if (z0 > 0) {
  //     ez = -ez;
  //     z0 = -z0;
  //   }
  //   auto x0 = dot(dr, ex);
  //   auto y0 = dot(dr, ey);
  //   auto x1 = x0 + lx;
  //   auto y1 = y0 + ly;
  //   auto v00 = vec3{x0, y0, z0};
  //   auto v01 = vec3{x0, y1, z0};
  //   auto v10 = vec3{x1, y0, z0};
  //   auto v11 = vec3{x1, y1, z0};
  //   auto n0 = normalize(cross(v00, v10));
  //   auto n1 = normalize(cross(v10, v11));
  //   auto n2 = normalize(cross(v11, v01));
  //   auto n3 = normalize(cross(v01, v00));
  //   auto g0 = psl::acos(-dot(n0, n1));
  //   auto g1 = psl::acos(-dot(n1, n2));
  //   auto g2 = psl::acos(-dot(n2, n3));
  //   auto g3 = psl::acos(-dot(n3, n0));
  //   auto b0 = n0.z;
  //   auto b1 = n2.z;
  //   auto k = 2 * Pi - g2 - g3;
  //   auto S = g0 + g1 - k;
  //   auto au = u[0] * S + k;
  //   auto fu = (psl::cos(au) * b0 - b1) / psl::sin(au);
  //   auto cu = 1 / psl::sqrt(fu * fu + b0 * b0) * psl::sign(fu);
  //   cu = psl::clamp(cu, -1.0f, 1.0f);
  //   auto xu = -(cu * z0) / psl::sqrt(1 - cu * cu);
  //   xu = psl::clamp(xu, x0, x1);
  //   auto d = psl::sqrt(xu * xu + z0 * z0);
  //   auto h0 = y0 / psl::sqrt(d * d + y0 * y0);
  //   auto h1 = y1 / psl::sqrt(d * d + y1 * y1);
  //   auto hv = h0 + u[1] * (h1 - h0), hv2 = hv * hv;
  //   auto yv = (hv2 < 1 - epsilon) ? (hv * d) / psl::sqrt(1 - hv2) : y1;
  //   ss.p = o + xu * ex + yv * ey + z0 * ez;
  //   ss.n = n;
  //   ss.uv = u;
  //   ss.w = normalize(ss.p - o, ss.distance);
  //   ss.pdf = 1.0f / S;
  //   return ss;
}
float Rect::pdf(const Ray& ray, vec3, vec3 ns) const {
  return psl::sqr(ray.tmax) / area() * absdot(ns, ray.d);

  //   auto p = position - ex * lx / 2 - ey * ly / 2;
  //   auto ez = cross(ex, ey);
  //   auto dr = p - ray.o;
  //   auto z0 = dot(dr, ez);
  //   if (z0 > 0) {
  //     ez = -ez;
  //     z0 = -z0;
  //   }
  //   auto x0 = dot(dr, ex);
  //   auto y0 = dot(dr, ey);
  //   auto x1 = x0 + lx;
  //   auto y1 = y0 + ly;
  //   auto v00 = vec3{x0, y0, z0};
  //   auto v01 = vec3{x0, y1, z0};
  //   auto v10 = vec3{x1, y0, z0};
  //   auto v11 = vec3{x1, y1, z0};
  //   auto n0 = normalize(cross(v00, v10));
  //   auto n1 = normalize(cross(v10, v11));
  //   auto n2 = normalize(cross(v11, v01));
  //   auto n3 = normalize(cross(v01, v00));
  //   auto g0 = psl::acos(-dot(n0, n1));
  //   auto g1 = psl::acos(-dot(n1, n2));
  //   auto g2 = psl::acos(-dot(n2, n3));
  //   auto g3 = psl::acos(-dot(n3, n0));
  //   auto k = 2 * Pi - g2 - g3;
  //   auto S = g0 + g1 - k;
  //   S = psl::max(S, epsilon);
  //   return 1.0f / S;
}
AABB Rect::get_aabb() const {
  AABB aabb;
  aabb.extend(position - ex * lx / 2 - ey * ly / 2);
  aabb.extend(position - ex * lx / 2 + ey * ly / 2);
  aabb.extend(position + ex * lx / 2 - ey * ly / 2);
  aabb.extend(position + ex * lx / 2 + ey * ly / 2);
  return aabb;
}

Cone::Cone(vec3 p, vec3 n, float r, float h)
    : bottom(p, n, r), p(p + n * h), n(normalize(n)), r(r), h(h) {
  A2 = sqr(r / h) + 1;
  A = psl::sqrt(A2);
  S = r / psl::sqrt(r * r + h * h);
}
bool Cone::hit(const Ray& ray) const {
  auto o = ray.o - p;
  const auto& d = ray.d;
  auto a = -A2 * sqr(dot(d, n)) + dot(d, d);
  auto b = 2 * (-A2 * dot(o, n) * dot(d, n) + dot(o, d));
  auto c = -A2 * sqr(dot(o, n)) + dot(o, o);
  auto tmax = ray.tmax;
  if (intersect_quadratic(a, b, c, ray.tmin, tmax) && dot(o + tmax * d, n) <= 0)
    return true;
  else
    return false;
  // return bottom.hit(ray);
}
bool Cone::intersect(Ray& ray, SurfaceInteraction& it) const {
  auto o = ray.o - p;
  const auto& d = ray.d;
  auto a = -A2 * sqr(dot(d, n)) + dot(d, d);
  auto b = 2 * (-A2 * dot(o, n) * dot(d, n) + dot(o, d));
  auto c = -A2 * sqr(dot(o, n)) + dot(o, o);
  auto tmax = ray.tmax;
  if (intersect_quadratic(a, b, c, ray.tmin, tmax) && dot(o + tmax * d, n) < 0) {
    ray.tmax = tmax;

    // auto ps = ray();t
    // auto l = length(ps - p) * A;
    // auto x = p - n * l;
    // it.n = normalize(ps - x);
    // it.p = x + it.n * l * S;

    // if (bottom.intersect(ray, it)) bottom.compute_surface_info(ray(), it);
    return true;
  }

  // if (bottom.intersect(ray, it)) {
  //   bottom.compute_surface_info(ray(), it);
  //   return true;
  // } else {
  return false;
  // }
}
void Cone::compute_surface_info(vec3 ps, SurfaceInteraction& it) const {
  auto l = length(ps - p) * A;
  auto x = p - n * l;
  it.n = normalize(ps - x);
  it.p = x + it.n * l * S;
}
psl::optional<ShapeSample> Cone::sample(vec3, vec2) const { return {}; }
float Cone::pdf(const Ray& ray, vec3, vec3 ns) const {
  return psl::sqr(ray.tmax) / area() * absdot(ns, ray.d);
}

bool Cylinder::hit(const Ray& ray) const {
  vec3 m = ray.o - p0;
  vec3 v = ray.d - dot(ray.d, n) * n;
  vec3 w = m - dot(m, n) * n;

  float a = dot(v, v);
  float b = 2 * dot(v, w);
  float c = dot(w, w) - r * r;

  float discriminant = b * b - 4 * a * c;

  if (discriminant < 0) return false;

  float sqrtDisc = std::sqrt(discriminant);
  float t = (-b - sqrtDisc) / (2 * a);
  if (t < ray.tmin) t = (-b + sqrtDisc) / (2 * a);
  if (t > ray.tmax) return false;
  auto hit_point = ray(t);

  vec3 projection = p0 + dot(hit_point - p0, n) * n;

  if (dot(projection - p0, n) < 0 || dot(projection - p1, n) > 0) return false;
  return true;
}
bool Cylinder::intersect(Ray& ray, SurfaceInteraction& it) const {
  vec3 m = ray.o - p0;
  vec3 v = ray.d - dot(ray.d, n) * n;
  vec3 w = m - dot(m, n) * n;

  float a = dot(v, v);
  float b = 2 * dot(v, w);
  float c = dot(w, w) - r * r;

  float discriminant = b * b - 4 * a * c;

  if (discriminant < 0) return false;

  float sqrtDisc = std::sqrt(discriminant);
  float t = (-b - sqrtDisc) / (2 * a);
  if (t < ray.tmin) t = (-b + sqrtDisc) / (2 * a);
  if (t > ray.tmax) return false;
  auto hit_point = ray(t);

  vec3 projection = p0 + dot(hit_point - p0, n) * n;

  if (dot(projection - p0, n) < 0 || dot(projection - p1, n) > 0) return false;

  ray.tmax = t;
  it.n = normalize(hit_point - projection);
  it.p = hit_point;
  return true;
}
void Cylinder::compute_surface_info(vec3, SurfaceInteraction&) const {
  // auto l = length(ps - p) * A;
  // auto x = p - n * l;
  // it.n = normalize(ps - x);
  // it.p = x + it.n * l * S;
}

Triangle::Triangle(vec3 v0, vec3 v1, vec3 v2, vec3 n) : v0(v0), v1(v1), v2(v2), n{normalize(n)} {
  if (this->n == vec3(0.0f)) this->n = vec3(0, 0, 1);
}
Triangle::Triangle(vec3 v0, vec3 v1, vec3 v2)
    : v0(v0), v1(v1), v2(v2), n{normalize(cross(v0 - v1, v0 - v2))} {
  if (n == vec3(0.0f)) n = vec3(0, 0, 1);
}
bool Triangle::hit(const Ray& ray, vec3 v0, vec3 v1, vec3 v2) {
  vec3 E1 = v1 - v0;
  vec3 E2 = v2 - v0;
  vec3 T = ray.o - v0;
  vec3 P = cross(ray.d, E2);
  vec3 Q = cross(T, E1);
  float D = dot(P, E1);
  if (D == 0.0f) return false;
  float t = dot(Q, E2) / D;
  if (t < ray.tmin || t > ray.tmax) return false;
  float u = dot(P, T) / D;
  if (u < 0.0f || u > 1.0f) return false;
  float v = dot(Q, ray.d) / D;
  if (v < 0.0f || v > 1.0f) return false;
  return u + v < 1.0f;
}
bool Triangle::intersect(Ray& ray, vec3 v0, vec3 v1, vec3 v2) {
  vec3 E1 = v1 - v0;
  vec3 E2 = v2 - v0;
  vec3 T = ray.o - v0;
  vec3 P = cross(ray.d, E2);
  vec3 Q = cross(T, E1);
  float D = dot(P, E1);
  if (D == 0.0f) return false;
  float t = dot(Q, E2) / D;
  if (t <= ray.tmin || t >= ray.tmax) return false;
  float u = dot(P, T) / D;
  if (u < 0.0f || u > 1.0f) return false;
  float v = dot(Q, ray.d) / D;
  if (v < 0.0f || v > 1.0f) return false;
  if (u + v > 1.0f) return false;
  ray.tmax = t;
  return true;
}
bool Triangle::hit(const Ray& ray) const { return hit(ray, v0, v1, v2); }
bool Triangle::intersect(Ray& ray, SurfaceInteraction&) const { return intersect(ray, v0, v1, v2); }
void Triangle::compute_surface_info(vec3 p, SurfaceInteraction& it) const {
  float u = dot(p - v0, v1 - v0);
  float v = dot(p - v0, v2 - v0);
  it.uv = vec2(u, v);
  it.p = lerp(it.uv[0], it.uv[1], v0, v1, v2);
  it.n = n;
}
psl::optional<ShapeSample> Triangle::sample(vec3 p, vec2 u) const {
  ShapeSample ss;
  if (u.x + u.y > 1.0f) u = vec2(1.0f) - u;
  ss.p = lerp(u.x, u.y, v0, v1, v2);
  ss.n = n;
  ss.uv = u;
  ss.w = normalize(ss.p - p, ss.distance);
  ss.pdf = psl::sqr(ss.distance) / psl::max(absdot(ss.w, ss.n) * area(), epsilon);
  return ss;
}
float Triangle::pdf(const Ray& ray, vec3, vec3 ns) const {
  return psl::sqr(ray.tmax) / (area() * absdot(ns, ray.d));
}
AABB Triangle::get_aabb() const {
  AABB aabb;
  aabb.extend(v0);
  aabb.extend(v1);
  aabb.extend(v2);
  return aabb;
}

Mesh::Mesh(psl::vector<vec3> vertices_, psl::vector<vec3u32> indices_, psl::vector<vec2> texcoords_,
           psl::vector<vec3> normals_)
    : vertices{MOVE(vertices_)},
      normals{MOVE(normals_)},
      texcoords{MOVE(texcoords_)},
      indices{MOVE(indices_)} {
  if (normals.size() != 0) CHECK_EQ(vertices.size(), normals.size());
  if (texcoords.size() != 0) CHECK_EQ(vertices.size(), texcoords.size());
};
bool Mesh::intersect(Ray& ray, SurfaceInteraction& it) const {
  static std::once_flag flag;
  std::call_once(flag, [&]() {
    auto bvh = new ShapeAccel(ShapeBVH());
    bvh->build(this);
    accel = bvh;
  });
  while (!accel);

  return accel->intersect(ray, it);
}
bool Mesh::hit(const Ray& ray, int index) const {
  DCHECK_LT(size_t(index), indices.size());
  auto face = indices[index];
  return Triangle::hit(ray, vertices[face[0]], vertices[face[1]], vertices[face[2]]);
}
bool Mesh::intersect(Ray& ray, int index) const {
  DCHECK_LT(size_t(index), indices.size());
  auto face = indices[index];
  return Triangle::intersect(ray, vertices[face[0]], vertices[face[1]], vertices[face[2]]);
}
void Mesh::compute_surface_info(vec3 p, SurfaceInteraction& it, int index) const {
  auto face = indices[index];
  auto v0 = vertices[face[0]], v1 = vertices[face[1]], v2 = vertices[face[2]];
  auto e1 = v1 - v0;
  auto e2 = v2 - v0;
  it.n = cross(e1, e2);
  auto tbn = inverse(mat3(e1, e2, it.n));
  it.uv = vec2(tbn * (p - v0));
  it.p = lerp(it.uv[0], it.uv[1], v0, v1, v2);
  if (normals.size())
    it.n = *normal_of(face, it.uv);
  else
    it.n = normalize(it.n);
  if (texcoords.size()) it.uv = *texcoord_of(face, it.uv);
}
AABB Mesh::get_aabb(size_t index) const {
  DCHECK_LT(index, indices.size());
  return Triangle(vertices[indices[index][0]], vertices[indices[index][1]],
                  vertices[indices[index][2]], vec3(0, 0, 1))
      .get_aabb();
}
AABB Mesh::get_aabb() const {
  auto aabb = AABB{};
  for (size_t i = 0; i < num_triangles(); i++) aabb.extend(get_aabb(i));
  return aabb;
}
Mesh& Mesh::apply(mat4 m) {
  for (auto& v : vertices) v = vec3(m * vec4(v, 1.0f));

  auto m3 = transpose(inverse(mat3(m)));
  for (auto& n : normals) n = normalize(m3 * n);
  return *this;
}

Mesh heightmap(const Array2d<float>& height_map) {
  auto width = height_map.size().x + 1;
  auto height = height_map.size().y + 1;
  auto p2i = [width](int x, int y) { return x + y * width; };
  auto vertices = psl::vector<vec3>(width * height);
  auto texcoords = psl::vector<vec2>(width * height);
  parallel_for(vec2i(width, height), [&](vec2i p) {
    auto x = p.x, y = p.y;
    vertices[p2i(x, y)].x = float(x - height_map.size().x / 2) / height_map.size().x;
    vertices[p2i(x, y)].z = float(y - height_map.size().y / 2) / height_map.size().y;
    texcoords[p2i(x, y)] = vec2(x, y) / height_map.size();
    auto n = 0;
    for (int xi = -1; xi <= 1; xi++)
      for (int yi = -1; yi <= 1; yi++)
        if (inside(vec2i(x + xi, y + yi), vec2i(0, 0), height_map.size())) {
          vertices[p2i(x, y)].y += height_map[vec2i(x + xi, y + yi)];
          n++;
        }
    vertices[p2i(x, y)].y /= n;
  });

  auto indices = psl::vector<vec3u32>(area(height_map.size()) * 2);
  parallel_for(height_map.size(), [&](vec2i p) {
    auto x = p.x, y = p.y;
    auto index = (x + y * height_map.size().x) * 2;
    indices[index] = vec3i(p2i(x, y), p2i(x + 1, y), p2i(x + 1, y + 1));
    indices[index + 1] = vec3i(p2i(x, y), p2i(x + 1, y + 1), p2i(x, y + 1));
  });
  return Mesh(MOVE(vertices), MOVE(indices), MOVE(texcoords));
}

Mesh heightmap(vec2i resolution, psl::function<float(vec2i)> height_function) {
  auto height_map = Array2df(resolution);
  parallel_for(resolution, [&](vec2i p) { height_map[p] = height_function(p); });
  return heightmap(height_map);
}
Mesh heightmap(vec2i resolution, psl::function<float(vec2)> height_function) {
  auto height_map = Array2df(resolution);
  parallel_for(resolution,
               [&](vec2i p) { height_map[p] = height_function((p + vec2(0.5f)) / resolution); });
  return heightmap(height_map);
}
bool SDF::hit(const Ray& ray) const {
  auto t = ray.tmin;
  for (int i = 0; i < 128; i++) {
    auto d = sdf(ray(t));
    if (d < 1e-5f) return true;
    if (t >= ray.tmax) return false;
    t += d;
  }

  return false;
}
bool SDF::intersect(Ray& ray, SurfaceInteraction&) const {
  auto t = ray.tmin;
  for (int i = 0; i < 128; i++) {
    auto d = sdf(ray(t));
    if (d < 1e-5f) {
      ray.tmax = t;
      return true;
    }
    if (t >= ray.tmax) return false;
    t += d;
  }

  return false;
}
void SDF::compute_surface_info(vec3 p, SurfaceInteraction& it) const {
  const auto ops = 1e-6f;
  auto dtdx = sdf(p + vec3(ops, 0.0f, 0.0f)) - sdf(p + vec3(-ops, 0.0f, 0.0f));
  auto dtdy = sdf(p + vec3(0.0f, ops, 0.0f)) - sdf(p + vec3(0.0f, -ops, 0.0f));
  auto dtdz = sdf(p + vec3(0.0f, 0.0f, ops)) - sdf(p + vec3(0.0f, 0.0f, -ops));
  it.p = p;
  it.n = normalize(vec3(dtdx, dtdy, dtdz));
}

CSGUnion::CSGUnion(Shape a, Shape b)
    : a(psl::make_shared<Shape>(MOVE(a))), b(psl::make_shared<Shape>(MOVE(b))) {}
bool CSGUnion::hit(const Ray& ray) const { return a->hit(ray) || b->hit(ray); }
bool CSGUnion::intersect(Ray& ray, SurfaceInteraction& it) const {
  auto intersect_a = a->intersect(ray, it);
  auto intersect_b = b->intersect(ray, it);
  if (intersect_a || intersect_b) {
    (intersect_b ? b : a)->compute_surface_info(ray(), it);
    return true;
  }
  return false;
}
AABB CSGUnion::get_aabb() const { return union_(a->get_aabb(), b->get_aabb()); }

CSGIntersection::CSGIntersection(Shape a, Shape b)
    : a(psl::make_shared<Shape>(MOVE(a))), b(psl::make_shared<Shape>(MOVE(b))) {}
bool CSGIntersection::hit(const Ray& ray) const {
  auto ray_a = ray, ray_b = ray;
  auto it_a = SurfaceInteraction(), it_b = SurfaceInteraction();
  while (true) {
    auto intersect_a = a->intersect_full(ray_a, it_a);
    auto intersect_b = b->intersect_full(ray_b, it_b);
    if (!intersect_a || !intersect_b) return false;
    if (ray_b.tmax < ray_a.tmax) {
      psl::swap(ray_a, ray_b);
      psl::swap(it_a, it_b);
    }
    if (it_b.is_outward(ray)) {
      return true;
    }
    it_a.n = face_same_hemisphere(it_a.n, ray.d);
    ray_a = ray_b = it_a.spawn_ray(ray.d, ray.tmax - ray_a.tmax);
  }
}
bool CSGIntersection::intersect(Ray& ray, SurfaceInteraction& it) const {
  auto ray_a = ray, ray_b = ray;
  auto it_a = it, it_b = it;
  while (true) {
    auto intersect_a = a->intersect_full(ray_a, it_a);
    auto intersect_b = b->intersect_full(ray_b, it_b);
    if (!intersect_a || !intersect_b) return false;
    if (ray_b.tmax < ray_a.tmax) {
      psl::swap(ray_a, ray_b);
      psl::swap(it_a, it_b);
    }
    if (it_b.is_outward(ray)) {
      ray = ray_a;
      it = it_a;
      return true;
    }
    it_a.n = face_same_hemisphere(it_a.n, ray.d);
    ray_a = ray_b = it_a.spawn_ray(ray.d, ray.tmax - ray_a.tmax);
  }
}
AABB CSGIntersection::get_aabb() const { return intersect_(a->get_aabb(), b->get_aabb()); }

CSGDifference::CSGDifference(Shape a, Shape b)
    : a(psl::make_shared<Shape>(MOVE(a))), b(psl::make_shared<Shape>(MOVE(b))) {}
bool CSGDifference::hit(Ray ray) const {
  auto ita = SurfaceInteraction(), itb = SurfaceInteraction();
  auto ra = ray, rb = ray;
  auto end = ray();
  for (auto i = 0; i < 1000; i++) {
    if (!a->intersect_full(ra, ita)) return false;
    if (!b->intersect_full(rb, itb)) {
      return true;
    }

    if (ra.tmax <= rb.tmax) {
      if (itb.is_inward(ray)) {
        return true;
      } else {
        ra = rb = ita.spawn_ray(ray.d, distance(ita.p, end));
      }
    } else {
      if (ita.is_outward(ray) && itb.is_outward(ray)) {
        return true;
      } else {
        ra = rb = itb.spawn_ray(ray.d, distance(itb.p, end));
      }
    }
  }
  return false;
}
bool CSGDifference::intersect(Ray& ray, SurfaceInteraction& it) const {
  auto ita = it, itb = it;
  auto ra = ray, rb = ray;
  auto end = ray();
  for (auto i = 0; i < 1000; i++) {
    if (!a->intersect_full(ra, ita)) return false;
    if (!b->intersect_full(rb, itb)) {
      it = ita;
      ray.tmax = distance(ray.o, it.p);
      // it.n = vec3(-1, -1, 1);
      return true;
    }

    if (ra.tmax <= rb.tmax) {
      if (itb.is_inward(ray)) {
        it = ita;
        ray.tmax = distance(ray.o, it.p);
        // it.n = vec3(1, -1, -1);
        return true;
      } else {
        ra = rb = ita.spawn_ray(ray.d, distance(ita.p, end));
      }
    } else {
      if (ita.is_outward(ray) && itb.is_outward(ray)) {
        it = itb;
        it.n *= -1;
        ray.tmax = distance(ray.o, it.p);
        // it.n = vec3(-1, 1, -1);
        return true;
      } else {
        ra = rb = itb.spawn_ray(ray.d, distance(itb.p, end));
      }
    }
  }
  return false;
}
AABB CSGDifference::get_aabb() const { return a->get_aabb(); }

// bool Shape::intersect(Ray& ray, SurfaceInteraction& it) const {
//   return dispatch([&]<typename T>(const T& x) {
//     if constexpr (psl::same_as<T, Mesh>) {
//       return x.intersect(ray, it);
//     } else {
//       if (x.intersect(ray, it)) {
//         x.compute_surface_info(it);
//         return true;
//       } else {
//         return false;
//       }
//     }
//   });
// }

void geometry_context(Context& ctx) {
  ctx.type<Ray>("Ray")
      .ctor<>()
      .member<&Ray::o>("o")
      .member<&Ray::d>("d")
      .member<&Ray::tmin>("tmin")
      .member<&Ray::tmax>("tmax");
  ctx.type<SurfaceInteraction>("SurfaceInteraction")
      .ctor<>()
      .member<&SurfaceInteraction::n>("n")
      .member<&SurfaceInteraction::p>("p")
      .member<&SurfaceInteraction::uv>("uv");
  ctx.type<psl::shared_ptr<Geometry>>("GeometryPtr");
  ctx.type<AABB>("AABB").ctor<vec3, vec3>().member<&AABB::lower>("lower").member<&AABB::upper>(
      "upper");
  ctx.type<OBB>("OBB").ctor<AABB, mat4>();
  ctx("Box") = [](vec3 lower, vec3 upper) { return AABB(lower, upper); };
  ctx("Box") = [](AABB aabb, mat4 transformation) { return OBB(aabb, transformation); };
  ctx("Box") = [](vec3 lower, vec3 upper, mat4 transformation) {
    return OBB(AABB(lower, upper), transformation);
  };
  ctx.type<Sphere>("Sphere").ctor<vec3, float>();
  ctx.type<Plane>("Plane").ctor<vec3, vec3>();
  ctx.type<Disk>("Disk").ctor<vec3, vec3, float>();
  ctx.type<Line>("Line").ctor<vec3, vec3, float>();
  ctx.type<Rect>("Rect").ctor<vec3, vec3, vec3>().ctor<vec3, vec3, vec3, bool>();
  ctx.type<Cone>("Cone").ctor<vec3, vec3, float, float>();
  ctx.type<Cylinder>("Cylinder").ctor<vec3, vec3, float>();
  ctx.type<Triangle>("Triangle").ctor<vec3, vec3, vec3>();
  ctx.type<SDF>("SDF")
      .ctor<vec3, vec3, psl::function<float(vec3)>>()
      .ctor<AABB, psl::function<float(vec3)>>();
  ctx.type<Mesh>("Mesh").method<&Mesh::apply>("apply");
  ctx.type<Shape>("Shape");
  ctx.type<CSGUnion>("CSGUnion").ctor<Shape, Shape>("+");
  ctx.type<CSGIntersection>("CSGIntersection").ctor<Shape, Shape>("*");
  ctx.type<CSGDifference>("CSGDifference").ctor<Shape, Shape>("-");
  ctx.type<Shape>()
      .ctor_variant<AABB, OBB, Sphere, Plane, Disk, Line, Cone, Cylinder, Triangle, Rect, Mesh, SDF,
                    CSGUnion, CSGIntersection, CSGDifference>();
  ctx.type<InstancedShape>("Instancing")
      .ctor<Mesh>()
      .method<overloaded<mat4, psl::shared_ptr<Material>>(&InstancedShape::add)>("add")
      .method<overloaded<mat4, Material>(&InstancedShape::add)>("add");
  ctx("heightmap") = led<overloaded<vec2i, psl::function<float(vec2)>>(heightmap)>;
}

}  // namespace pine