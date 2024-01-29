#include <pine/core/geometry.h>
#include <pine/core/context.h>

namespace pine {

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
  if (!is_valid()) {
    lower = min(lower, lower - vec3(1e-4f));
    upper = max(upper, upper + vec3(1e-4f));
  }
}
void AABB::extend(const AABB& aabb) {
  lower = min(lower, aabb.lower);
  upper = max(upper, aabb.upper);
  if (!is_valid()) {
    lower = min(lower, lower - vec3(1e-4f));
    upper = max(upper, upper + vec3(1e-4f));
  }
}
psl::pair<AABB, AABB> AABB::split_half(int axis) const {
  CHECK_RANGE(axis, 0, 2);
  auto left = *this;
  auto right = *this;
  left.upper[axis] = centroid(axis);
  right.lower[axis] = centroid(axis);
  return {left, right};
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
psl::optional<vec3> extend_to(Ray ray, int axis, float p) {
  auto t = (p - ray.o[axis]) / ray.d[axis];
  if (!(t > ray.tmin))
    return psl::nullopt;
  if (!(t < ray.tmax))
    return psl::nullopt;
  return ray(t);
}
vec2 project(vec3 v, int axis) {
  return {v[(axis + 1) % 3], v[(axis + 2) % 3]};
}

float Sphere::compute_t(vec3 ro, vec3 rd, float tmin, vec3 p, float r) {
  float a = dot(rd, rd);
  float b = 2 * dot(ro - p, rd);
  float c = dot(ro, ro) + dot(p, p) - 2 * dot(ro, p) - r * r;
  float d = b * b - 4 * a * c;
  if (d <= 0.0f)
    return -1.0f;
  d = psl::sqrt(d);
  float t = (-b - d) / (2 * a);
  if (t < tmin)
    t = (-b + d) / (2 * a);
  return t;
}
bool Sphere::hit(const Ray& ray) const {
  float t = compute_t(ray.o, ray.d, ray.tmin, c, r);
  return t > ray.tmin && t < ray.tmax;
}
bool Sphere::intersect(Ray& ray, Interaction& it) const {
  float t = compute_t(ray.o, ray.d, ray.tmin, c, r);
  if (t < ray.tmin)
    return false;
  if (t > ray.tmax)
    return false;
  ray.tmax = t;
  it.n = normalize(ray(t) - this->c);
  it.p = this->c + it.n * r;
  auto [phi, theta] = cartesian_to_spherical(it.n);
  it.uv = vec2(phi, theta);
  return true;
}
ShapeSample Sphere::sample(vec3 p, vec2 u) const {
  auto ss = ShapeSample{};
  ss.n = uniform_sphere(u);
  ss.p = offset_ray_origin(c + r * ss.n, ss.n);
  ss.uv = u;
  ss.w = normalize(ss.p - p, ss.distance);
  ss.pdf = psl::sqr(ss.distance) / (absdot(ss.w, ss.n) * area());
  return ss;
}
float Sphere::pdf(const Interaction& it, const Ray& ray, vec3) const {
  return psl::sqr(ray.tmax) / (area() * absdot(it.n, -ray.d));
  ;
}
AABB Sphere::get_aabb() const {
  return {c - vec3(r), c + vec3(r)};
}

bool Plane::hit(const Ray& ray) const {
  float t = (dot(position, n) - dot(ray.o, n)) / dot(ray.d, n);
  if (t <= ray.tmin)
    return false;
  return t < ray.tmax;
}
bool Plane::intersect(Ray& ray, Interaction& it) const {
  float t = (dot(position, n) - dot(ray.o, n)) / dot(ray.d, n);
  if (t < ray.tmin)
    return false;
  if (t >= ray.tmax)
    return false;
  ray.tmax = t;
  it.p = ray.o + t * ray.d;
  it.n = n;
  it.uv = vec2(dot(it.p - position, u), dot(it.p - position, v));
  it.p = position + it.uv.x * u + it.uv.y * v;
  return true;
}
AABB Plane::get_aabb() const {
  return {vec3(-10.0f), vec3(10.0f)};
}
ShapeSample Plane::sample(vec3 p, vec2 u2) const {
  auto ss = ShapeSample{};
  auto p_sphere = uniform_hemisphere(u2);
  auto l = absdot(p - position, n);
  auto ex = l * p_sphere.x / p_sphere.z;
  auto ey = l * p_sphere.y / p_sphere.z;
  ss.p = position + u * ex + v * ey;
  ss.n = n;
  ss.uv = vec2{ex, ey};
  ss.w = normalize(ss.p - p, ss.distance);
  ss.pdf = 1.0f / (2 * pi);
  return ss;
}
float Plane::pdf(const Interaction&, const Ray&, vec3) const {
  return 1.0f / (2 * pi);
}

bool Disk::hit(const Ray& ray) const {
  float t = (dot(position, n) - dot(ray.o, n)) / dot(ray.d, n);
  if (t < ray.tmin)
    return false;
  if (t >= ray.tmax)
    return false;
  vec3 p = ray.o + t * ray.d - position;
  if (length_squared(p) > psl::sqr(r))
    return false;
  return true;
}
bool Disk::intersect(Ray& ray, Interaction& it) const {
  float t = (dot(position, n) - dot(ray.o, n)) / dot(ray.d, n);
  if (t < ray.tmin)
    return false;
  if (t >= ray.tmax)
    return false;
  vec3 p = ray.o + t * ray.d - position;
  if (length_squared(p) > psl::sqr(r))
    return false;

  ray.tmax = t;
  it.p = ray.o + t * ray.d;
  it.n = n;
  it.uv = vec2(length(p), phi2pi(p.x, p.z));
  return true;
}
ShapeSample Disk::sample(vec3 p, vec2 u2) const {
  auto ss = ShapeSample{};
  auto uv = sample_disk_concentric(u2);
  ss.p = position + r * u * uv[0] + r * v * uv[1];
  ss.n = n;
  ss.uv = u2;
  ss.w = normalize(ss.p - p, ss.distance);
  ss.pdf = psl::sqr(ss.distance) / (absdot(ss.w, ss.n) * area());
  return ss;
}
float Disk::pdf(const Interaction& it, const Ray& ray, vec3) const {
  return psl::sqr(ray.tmax) / (area() * absdot(it.n, -ray.d));
}
AABB Disk::get_aabb() const {
  return {position - vec3(r, 0.0f, r), position + vec3(r, 0.0f, r)};
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
Line::Line(vec3 p0, vec3 p1, float thickness)
    : p0(p0),
      p1(p1),
      tbn(coordinate_system(normalize(p1 - p0))),
      thickness(thickness),
      len{length(p1 - p0)} {};
bool Line::intersect(Ray& ray, Interaction& it) const {
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

  if (D > thickness)
    return false;

  ray.tmax = z;
  it.p = ray(z);
  auto lt = dot(it.p - this->p0, tbn.z);
  auto lp = lerp(lt, this->p0, this->p1);
  it.n = normalize(it.p - lp);

  return true;
}
ShapeSample Line::sample(vec3 p, vec2 u2) const {
  auto ss = ShapeSample{};
  auto phi = u2[1] * 2 * pi;
  ss.p =
      lerp(u2[0], p0, p1) + thickness * psl::cos(phi) * tbn.x + thickness * psl::sin(phi) * tbn.y;
  ss.n = psl::cos(phi) * tbn.x + psl::sin(phi) * tbn.y;
  ss.uv = u2;
  ss.w = normalize(ss.p - p, ss.distance);
  ss.pdf = psl::sqr(ss.distance) / (absdot(ss.w, ss.n) * area());
  return ss;
}
float Line::pdf(const Interaction& it, const Ray& ray, vec3) const {
  return psl::sqr(ray.tmax) / (area() * absdot(it.n, -ray.d));
}
AABB Line::get_aabb() const {
  AABB aabb;
  aabb.extend(p0 - vec3(thickness));
  aabb.extend(p1 - vec3(thickness));
  aabb.extend(p0 + vec3(thickness));
  aabb.extend(p1 + vec3(thickness));
  return aabb;
}

Rect::Rect(vec3 position, vec3 ex, vec3 ey)
    : position(position),
      ex(normalize(ex)),
      ey(normalize(ey)),
      n(normalize(cross(ex, ey))),
      lx(length(ex)),
      ly(length(ey)){};
Rect Rect::from_vertex(vec3 v0, vec3 v1, vec3 v2) {
  auto ex = v1 - v0;
  auto ey = v2 - v0;
  return Rect{v0 + ex / 2 + ey / 2, ex, ey};
}
Rect Rect::apply(mat4 m) const {
  auto v0 = position - ex * lx / 2 - ey * ly / 2;
  auto v1 = v0 + ex * lx;
  auto v2 = v0 + ey * ly;
  v0 = vec3{m * vec4{v0, 1.0f}};
  v1 = vec3{m * vec4{v1, 1.0f}};
  v2 = vec3{m * vec4{v2, 1.0f}};
  return from_vertex(v0, v1, v2);
}
bool Rect::hit(const Ray& ray) const {
  float t = (dot(position, n) - dot(ray.o, n)) / dot(ray.d, n);
  if (t < ray.tmin)
    return false;
  if (t >= ray.tmax)
    return false;
  vec3 p = ray.o + t * ray.d;
  float u = dot(p - position, ex) / lx;
  if (u < -0.5f || u > 0.5f)
    return false;
  float v = dot(p - position, ey) / ly;
  if (v < -0.5f || v > 0.5f)
    return false;
  return true;
}
bool Rect::intersect(Ray& ray, Interaction& it) const {
  float t = (dot(position, n) - dot(ray.o, n)) / dot(ray.d, n);
  if (psl::isnan(t))
    return false;
  if (t < ray.tmin)
    return false;
  if (t >= ray.tmax)
    return false;
  vec3 p = ray.o + t * ray.d;
  float u = dot(p - position, ex) / lx;
  if (u < -0.5f || u > 0.5f)
    return false;
  float v = dot(p - position, ey) / ly;
  if (v < -0.5f || v > 0.5f)
    return false;
  ray.tmax = t;
  it.p = position + lx * ex * u + ly * ey * v;
  it.n = n;
  it.uv = vec2(u, v) + vec2(0.5f);
  return true;
}
ShapeSample Rect::sample(vec3 o, vec2 u) const {
  auto ss = ShapeSample{};
  ss.p = position + (u[0] - 0.5f) * ex * lx + (u[1] - 0.5f) * ey * ly;
  ss.n = n;
  ss.uv = u;
  ss.w = normalize(ss.p - o, ss.distance);
  ss.pdf =
      psl::max(psl::sqr(ss.distance), epsilon) / (psl::max(absdot(ss.w, ss.n), epsilon) * area());
  return ss;

  auto p = position - ex * lx / 2 - ey * ly / 2;
  auto ez = cross(ex, ey);
  auto dr = p - o;
  auto z0 = dot(dr, ez);
  if (z0 > 0) {
    ez = -ez;
    z0 = -z0;
  }
  auto x0 = dot(dr, ex);
  auto y0 = dot(dr, ey);
  auto x1 = x0 + lx;
  auto y1 = y0 + ly;
  auto v00 = vec3{x0, y0, z0};
  auto v01 = vec3{x0, y1, z0};
  auto v10 = vec3{x1, y0, z0};
  auto v11 = vec3{x1, y1, z0};
  auto n0 = normalize(cross(v00, v10));
  auto n1 = normalize(cross(v10, v11));
  auto n2 = normalize(cross(v11, v01));
  auto n3 = normalize(cross(v01, v00));
  auto g0 = psl::acos(-dot(n0, n1));
  auto g1 = psl::acos(-dot(n1, n2));
  auto g2 = psl::acos(-dot(n2, n3));
  auto g3 = psl::acos(-dot(n3, n0));
  auto b0 = n0.z;
  auto b1 = n2.z;
  auto k = 2 * pi - g2 - g3;
  auto S = g0 + g1 - k;
  auto au = u[0] * S + k;
  auto fu = (psl::cos(au) * b0 - b1) / psl::sin(au);
  auto cu = 1 / psl::sqrt(fu * fu + b0 * b0) * psl::sign(fu);
  cu = psl::clamp(cu, -1.0f, 1.0f);
  auto xu = -(cu * z0) / psl::sqrt(1 - cu * cu);
  xu = psl::clamp(xu, x0, x1);
  auto d = psl::sqrt(xu * xu + z0 * z0);
  auto h0 = y0 / psl::sqrt(d * d + y0 * y0);
  auto h1 = y1 / psl::sqrt(d * d + y1 * y1);
  auto hv = h0 + u[1] * (h1 - h0), hv2 = hv * hv;
  auto yv = (hv2 < 1 - epsilon) ? (hv * d) / psl::sqrt(1 - hv2) : y1;
  ss.p = o + xu * ex + yv * ey + z0 * ez;
  ss.n = n;
  ss.uv = u;
  ss.w = normalize(ss.p - o, ss.distance);
  ss.pdf = 1.0f / S;
  return ss;
}
float Rect::pdf(const Interaction& it, const Ray& ray, vec3) const {
  return psl::max(psl::sqr(ray.tmax), epsilon) / psl::max(area() * absdot(it.n, -ray.d), epsilon);

  auto p = position - ex * lx / 2 - ey * ly / 2;
  auto ez = cross(ex, ey);
  auto dr = p - ray.o;
  auto z0 = dot(dr, ez);
  if (z0 > 0) {
    ez = -ez;
    z0 = -z0;
  }
  auto x0 = dot(dr, ex);
  auto y0 = dot(dr, ey);
  auto x1 = x0 + lx;
  auto y1 = y0 + ly;
  auto v00 = vec3{x0, y0, z0};
  auto v01 = vec3{x0, y1, z0};
  auto v10 = vec3{x1, y0, z0};
  auto v11 = vec3{x1, y1, z0};
  auto n0 = normalize(cross(v00, v10));
  auto n1 = normalize(cross(v10, v11));
  auto n2 = normalize(cross(v11, v01));
  auto n3 = normalize(cross(v01, v00));
  auto g0 = psl::acos(-dot(n0, n1));
  auto g1 = psl::acos(-dot(n1, n2));
  auto g2 = psl::acos(-dot(n2, n3));
  auto g3 = psl::acos(-dot(n3, n0));
  auto k = 2 * pi - g2 - g3;
  auto S = g0 + g1 - k;
  S = psl::max(S, epsilon);
  return 1.0f / S;
}
AABB Rect::get_aabb() const {
  AABB aabb;
  aabb.extend(position + ex * lx / 2);
  aabb.extend(position - ex * lx / 2);
  aabb.extend(position + ey * ly / 2);
  aabb.extend(position - ey * ly / 2);
  return aabb;
}

bool Triangle::hit(const Ray& ray, vec3 v0, vec3 v1, vec3 v2) {
  vec3 E1 = v1 - v0;
  vec3 E2 = v2 - v0;
  vec3 T = ray.o - v0;
  vec3 P = cross(ray.d, E2);
  vec3 Q = cross(T, E1);
  float D = dot(P, E1);
  if (D == 0.0f)
    return false;
  float t = dot(Q, E2) / D;
  if (t < ray.tmin || t > ray.tmax)
    return false;
  float u = dot(P, T) / D;
  if (u < 0.0f || u > 1.0f)
    return false;
  float v = dot(Q, ray.d) / D;
  if (v < 0.0f || v > 1.0f)
    return false;
  return u + v < 1.0f;
}
bool Triangle::intersect(Ray& ray, Interaction& it, vec3 v0, vec3 v1, vec3 v2) {
  vec3 E1 = v1 - v0;
  vec3 E2 = v2 - v0;
  vec3 T = ray.o - v0;
  vec3 P = cross(ray.d, E2);
  vec3 Q = cross(T, E1);
  float D = dot(P, E1);
  if (D == 0.0f)
    return false;
  float t = dot(Q, E2) / D;
  if (t <= ray.tmin || t >= ray.tmax)
    return false;
  float u = dot(P, T) / D;
  if (u < 0.0f || u > 1.0f)
    return false;
  float v = dot(Q, ray.d) / D;
  if (v < 0.0f || v > 1.0f)
    return false;
  if (u + v > 1.0f)
    return false;
  ray.tmax = t;
  it.uv = vec2(u, v);
  return true;
}
bool Triangle::intersect(Ray& ray, Interaction& it) const {
  bool hit = intersect(ray, it, v0, v1, v2);
  if (hit) {
    it.p = lerp(it.uv[0], it.uv[1], v0, v1, v2);
    it.n = n;
  }
  return hit;
}
ShapeSample Triangle::sample(vec3 p, vec2 u) const {
  ShapeSample ss;
  if (u.x + u.y > 1.0f)
    u = vec2(1.0f) - u;
  ss.p = lerp(u.x, u.y, v0, v1, v2);
  ss.n = n;
  ss.uv = u;
  ss.w = normalize(ss.p - p, ss.distance);
  ss.pdf = psl::sqr(ss.distance) / (absdot(ss.w, ss.n) * area());
  return ss;
}
float Triangle::pdf(const Interaction& it, const Ray& ray, vec3) const {
  return psl::sqr(ray.tmax) / (area() * absdot(it.n, -ray.d));
}
AABB Triangle::get_aabb() const {
  AABB aabb;
  aabb.extend(v0);
  aabb.extend(v1);
  aabb.extend(v2);
  return aabb;
}

TriangleMesh::TriangleMesh(psl::vector<vec3> vertices_, psl::vector<vec3u32> indices_,
                           psl::vector<vec2> texcoords_, psl::vector<vec3> normals_)
    : vertices{psl::move(vertices_)},
      normals{psl::move(normals_)},
      texcoords{psl::move(texcoords_)},
      indices{psl::move(indices_)} {
  if (normals.size() != 0)
    CHECK_EQ(vertices.size(), normals.size());
  if (texcoords.size() != 0)
    CHECK_EQ(vertices.size(), texcoords.size());
};
bool TriangleMesh::hit(const Ray& ray, int index) const {
  DCHECK_LT(size_t(index), indices.size());
  auto face = indices[index];
  return Triangle::hit(ray, vertices[face[0]], vertices[face[1]], vertices[face[2]]);
}
bool TriangleMesh::intersect(Ray& ray, Interaction& it, int index) const {
  DCHECK_LT(size_t(index), indices.size());
  auto face = indices[index];
  auto v0 = vertices[face[0]], v1 = vertices[face[1]], v2 = vertices[face[2]];
  auto hit = Triangle::intersect(ray, it, v0, v1, v2);
  if (hit) {
    it.p = lerp(it.uv[0], it.uv[1], v0, v1, v2);
    it.n = normalize(cross(v0 - v1, v0 - v2));
    if (normals.size())
      it.n =
          normalize(lerp(it.uv[0], it.uv[1], normals[face[0]], normals[face[1]], normals[face[2]]));
    if (texcoords.size())
      it.uv = lerp(it.uv[0], it.uv[1], texcoords[face[0]], texcoords[face[1]], texcoords[face[2]]);
  }
  return hit;
}
AABB TriangleMesh::get_aabb(size_t index) const {
  DCHECK_LT(index, indices.size());
  return Triangle(vertices[indices[index][0]], vertices[indices[index][1]],
                  vertices[indices[index][2]], {})
      .get_aabb();
}
AABB TriangleMesh::get_aabb() const {
  auto aabb = AABB{};
  for (size_t i = 0; i < num_triangles(); i++)
    aabb.extend(get_aabb(i));
  return aabb;
}

TriangleMesh height_map_to_mesh(const Array2d<float>& height_map) {
  auto width = height_map.size().x + 1;
  auto p2i = [width](int x, int y) { return y * width + x; };
  auto vertices = psl::vector<vec3>(width * (height_map.size().y + 1));
  for (int x = 0; x <= height_map.size().x; x++)
    for (int y = 0; y <= height_map.size().y; y++) {
      auto n = 0;
      vertices[p2i(x, y)].x = float(x - height_map.size().x / 2) / height_map.size().x;
      vertices[p2i(x, y)].z = float(y - height_map.size().y / 2) / height_map.size().y;
      for (int xi = -1; xi <= 1; xi++)
        for (int yi = -1; yi <= 1; yi++) {
          if (inside(vec2i(x + xi, y + yi), vec2i(0, 0), height_map.size())) {
            vertices[p2i(x, y)].y += height_map[vec2i(x + xi, y + yi)];
            n++;
          }
        }
      vertices[p2i(x, y)].y /= n;
    }

  auto indices = psl::vector<vec3u32>();
  for (int x = 0; x < height_map.size().x; x++)
    for (int y = 0; y < height_map.size().y; y++) {
      indices.push_back(vec3i(p2i(x, y), p2i(x + 1, y), p2i(x + 1, y + 1)));
      indices.push_back(vec3i(p2i(x, y), p2i(x + 1, y + 1), p2i(x, y + 1)));
    }
  return TriangleMesh(vertices, indices);
}

bool Geometry::intersect(Ray& ray, Interaction& it) const {
  auto hit = shape.intersect(ray, it);
  if (hit) {
    it.material = material.get();
    it.geometry = this;
  }
  return hit;
}
ShapeSample Geometry::sample(vec3 p, vec2 u) const {
  return shape.sample(p, u);
}
float Geometry::pdf(const Interaction& it, const Ray& ray, vec3 n) const {
  return shape.pdf(it, ray, n);
}

void geometry_context(Context& ctx) {
  ctx.type<Sphere>("Sphere").ctor<vec3, float>();
  ctx.type<Plane>("Plane").ctor<vec3, vec3>();
  ctx.type<Disk>("Disk").ctor<vec3, vec3, float>();
  ctx.type<Line>("Line").ctor<vec3, vec3, float>();
  ctx.type<Rect>("Rect").ctor<vec3, vec3, vec3>();
  ctx.type<Triangle>("Triangle").ctor<vec3, vec3, vec3>();
  ctx.type<TriangleMesh>("TriangleMesh").method("apply", &TriangleMesh::apply);
  ctx.type<Shape>("Shape").ctor_variant<Sphere, Plane, Disk, Line, Triangle, Rect, TriangleMesh>();
  ctx("height_map_to_mesh") = height_map_to_mesh;
}

}  // namespace pine