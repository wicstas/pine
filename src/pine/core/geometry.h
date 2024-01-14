#pragma once

#include <pine/core/interaction.h>
#include <pine/core/material.h>
#include <pine/core/ray.h>

#include <pine/psl/variant.h>

namespace pine {

struct RayOctant {
  RayOctant(const Ray& ray)
      : octantx3{psl::signbit(ray.d[0]) * 3, psl::signbit(ray.d[1]) * 3,
                 psl::signbit(ray.d[2]) * 3},
        invDir(safe_rcp(ray.d)),
        negOrgDivDir(-ray.o * invDir) {
  }

  int octantx3[3];
  vec3 invDir, negOrgDivDir;
};

struct AABB {
  AABB() = default;
  AABB(vec3 lower, vec3 upper) : lower(lower), upper(upper){};
  AABB(vec3 p) : lower(p), upper(p){};

  int MaxDim() const {
    vec3 diagonal = upper - lower;
    if (diagonal.x > diagonal.y)
      return diagonal.x > diagonal.z ? 0 : 2;
    else
      return diagonal.y > diagonal.z ? 1 : 2;
  }
  vec3 Centroid() const {
    return (lower + upper) / 2;
  }
  float Centroid(int dim) const {
    return (lower[dim] + upper[dim]) / 2;
  }
  vec3 Diagonal() const {
    return max(upper - lower, vec3(0.0f));
  }
  vec3 Offset(vec3 p) const;
  float Offset(float p, int dim) const;
  float SurfaceArea() const;
  void extend(vec3 p);
  void extend(const AABB& aabb);
  psl::pair<AABB, AABB> split_half(int axis) const;
  friend AABB extend(AABB l, vec3 r) {
    l.extend(r);
    return l;
  }
  friend AABB Union(AABB l, const AABB& r) {
    l.extend(r);
    return l;
  }
  friend AABB Intersection(const AABB& l, const AABB& r) {
    AABB ret;
    ret.lower = max(l.lower, r.lower);
    ret.upper = min(l.upper, r.upper);
    return ret;
  }
  bool is_valid() const {
    return (upper.x > lower.x) && (upper.y > lower.y) && (upper.z > lower.z);
  }
  bool is_valid(int dim) const {
    return upper[dim] > lower[dim];
  }

  bool hit(const Ray& ray) const;
  bool hit(Ray ray, float& tmin, float& tmax) const;

  PINE_ALWAYS_INLINE bool hit(const RayOctant& r, float tmin, float tmax) const {
    return hit(r, tmin, &tmax);
  }
  PINE_ALWAYS_INLINE bool hit(const RayOctant& r, float tmin, float* tmax) const {
    const float* p = &lower[0];
    float tmin0 = p[0 + r.octantx3[0]] * r.invDir[0] + r.negOrgDivDir[0];
    float tmin1 = p[1 + r.octantx3[1]] * r.invDir[1] + r.negOrgDivDir[1];
    float tmin2 = p[2 + r.octantx3[2]] * r.invDir[2] + r.negOrgDivDir[2];

    float tmax0 = p[3 - r.octantx3[0]] * r.invDir[0] + r.negOrgDivDir[0];
    float tmax1 = p[4 - r.octantx3[1]] * r.invDir[1] + r.negOrgDivDir[1];
    float tmax2 = p[5 - r.octantx3[2]] * r.invDir[2] + r.negOrgDivDir[2];

    tmin = psl::max(psl::max(psl::max(tmin0, tmin1), tmin2), tmin);
    *tmax = psl::min(psl::min(psl::min(tmax0, tmax1), tmax2), *tmax);
    return tmin <= *tmax;
  }

  int face_hit_count(const Ray& ray) const;

  vec3 lower = vec3{FloatMax};
  vec3 upper = vec3{-FloatMax};
};

struct ShapeSample {
  vec3 p;
  vec3 n;
  vec2 uv;
  vec3 w;
  float distance = 0.0f;
  float pdf = 0.0f;
};

struct Plane {
  Plane() = default;
  Plane(vec3 position, vec3 normal) : position(position), n(normal) {
    mat3 tbn = coordinate_system(normal);
    u = tbn.x;
    v = tbn.y;
  };

  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, Interaction& it) const;
  AABB get_aabb() const;
  float area() const {
    return FloatMax;
  }
  ShapeSample sample(vec3 p, vec2 u) const;
  float pdf(const Interaction& it, const Ray& ray, vec3 n) const;

private:
  vec3 position;
  vec3 n;
  vec3 u, v;
};

struct Sphere {
  Sphere() = default;
  Sphere(vec3 position, float radius) : c(position), r(radius){};

  static float ComputeT(vec3 ro, vec3 rd, float tmin, vec3 p, float r);
  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, Interaction& it) const;
  AABB get_aabb() const;
  float area() const {
    return 4 * pi * r * r;
  }
  ShapeSample sample(vec3, vec2 u) const;
  float pdf(const Interaction& it, const Ray& ray, vec3 n) const;

private:
  vec3 c;
  float r;
};

struct Disk {
  Disk() = default;
  Disk(vec3 position, vec3 normal, float r) : position(position), n(normal), r(r) {
    mat3 tbn = coordinate_system(normal);
    u = tbn.x;
    v = tbn.y;
  };

  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, Interaction& it) const;
  AABB get_aabb() const;
  float area() const {
    return pi * r * r;
  }
  ShapeSample sample(vec3, vec2) const;
  float pdf(const Interaction& it, const Ray& ray, vec3 n) const;

  vec3 position;
  vec3 n;
  vec3 u, v;
  float r;
};

struct Line {
  Line() = default;
  Line(vec3 p0, vec3 p1, float thickness)
      : p0(p0), p1(p1), thickness(thickness), len{length(p1 - p0)} {
    mat3 tbn = coordinate_system(normalize(p1 - p0));
    u = tbn.x;
    v = tbn.y;
  };

  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, Interaction& it) const;
  AABB get_aabb() const;
  float area() const {
    return thickness * 2 * pi * len;
  }
  ShapeSample sample(vec3, vec2) const;
  float pdf(const Interaction& it, const Ray& ray, vec3 n) const;

  vec3 p0, p1;
  vec3 u, v;
  float thickness;
  float len;
};

struct Rect {
  Rect() = default;
  Rect(vec3 position, vec3 ex, vec3 ey)
      : position(position),
        ex(normalize(ex)),
        ey(normalize(ey)),
        n(normalize(cross(ex, ey))),
        lx(length(ex)),
        ly(length(ey)){};
  static Rect from_vertex(vec3 v0, vec3 v1, vec3 v2) {
    auto ex = v1 - v0;
    auto ey = v2 - v0;
    return Rect{v0 + ex / 2 + ey / 2, ex, ey};
  }
  Rect apply(mat4 m) const {
    auto v0 = position - ex * lx / 2 - ey * ly / 2;
    auto v1 = v0 + ex * lx;
    auto v2 = v0 + ey * ly;
    v0 = vec3{m * vec4{v0, 1.0f}};
    v1 = vec3{m * vec4{v1, 1.0f}};
    v2 = vec3{m * vec4{v2, 1.0f}};
    return from_vertex(v0, v1, v2);
  }

  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, Interaction& it) const;
  AABB get_aabb() const;
  float area() const {
    return lx * ly;
  }
  ShapeSample sample(vec3, vec2 u) const;
  float pdf(const Interaction& it, const Ray& ray, vec3 n) const;

  vec3 position, ex, ey, n;
  float lx, ly;
};

struct Triangle {
  Triangle() = default;
  Triangle(vec3 v0, vec3 v1, vec3 v2, vec3 n) : v0(v0), v1(v1), v2(v2), n{n} {};
  Triangle(vec3 v0, vec3 v1, vec3 v2)
      : v0(v0), v1(v1), v2(v2), n{normalize(cross(v0 - v1, v0 - v2))} {};

  static bool hit(const Ray& ray, vec3 v0, vec3 v1, vec3 v2);
  static bool intersect(Ray& ray, Interaction& it, vec3 v0, vec3 v1, vec3 v2);

  bool hit(const Ray& ray) const {
    return hit(ray, v0, v1, v2);
  }
  bool intersect(Ray& ray, Interaction& it) const;

  AABB get_aabb() const;
  float area() const {
    return length(cross(v1 - v0, v2 - v0)) / 2;
  }
  ShapeSample sample(vec3, vec2 u) const;
  float pdf(const Interaction& it, const Ray& ray, vec3 n) const;

  vec3 v0, v1, v2;
  vec3 n;
};

struct TriangleMesh {
  TriangleMesh() = default;
  TriangleMesh(psl::vector<vec3> vertices, psl::vector<uint32_t> indices,
               psl::vector<vec2> texcoords = {}, psl::vector<vec3> normals = {});

  bool hit(const Ray&) const {
    PINE_UNREACHABLE;
  }
  bool intersect(Ray&, Interaction&) const {
    PINE_UNREACHABLE;
  }
  AABB get_aabb() const;
  float area() const {
    PINE_UNREACHABLE;
  }
  ShapeSample sample(vec3, vec2) const {
    PINE_UNREACHABLE;
  }
  float pdf(const Interaction&, const Ray&, vec3) const {
    PINE_UNREACHABLE;
  }

  bool hit(const Ray& ray, int index) const;
  bool intersect(Ray& ray, Interaction& it, int index) const;

  size_t num_triangles() const {
    return indices.size() / 3;
  }
  AABB get_aabb(size_t index) const;

  TriangleMesh apply(mat4 m) const {
    auto copy = *this;
    for (auto& v : copy.vertices)
      v = vec3{m * vec4{v, 1.0f}};
    return copy;
  }

private:
  psl::vector<vec3> vertices;
  psl::vector<vec3> normals;
  psl::vector<vec2> texcoords;
  psl::vector<uint32_t> indices;
};

struct Shape : psl::Variant<Sphere, Plane, Triangle, Rect, Disk, Line, TriangleMesh> {
  using Variant::Variant;

  bool hit(const Ray& ray) const {
    return dispatch([&](auto&& x) { return x.hit(ray); });
  }
  bool intersect(Ray& ray, Interaction& it) const {
    return dispatch([&](auto&& x) { return x.intersect(ray, it); });
  }
  AABB get_aabb() const {
    return dispatch([&](auto&& x) { return x.get_aabb(); });
  }
  float area() const {
    return dispatch([&](auto&& x) { return x.area(); });
  }
  ShapeSample sample(vec3 p, vec2 u) const {
    return dispatch([&](auto&& x) { return x.sample(p, u); });
  }
  float pdf(const Interaction& it, const Ray& ray, vec3 n) const {
    return dispatch([&](auto&& x) { return x.pdf(it, ray, n); });
  }
};

struct Geometry {
  Geometry() = default;
  Geometry(Shape shape, psl::shared_ptr<Material> material)
      : shape{psl::move(shape)}, material{psl::move(material)} {
  }

  bool hit(const Ray& ray) const {
    return shape.hit(ray);
  }
  bool intersect(Ray& ray, Interaction& it) const;
  float area() const {
    return shape.area();
  }
  AABB get_aabb() const {
    return shape.get_aabb();
  }
  ShapeSample sample(vec3 p, vec2 u) const;
  float pdf(const Interaction& it, const Ray& ray, vec3 n) const;

  Shape shape;
  psl::shared_ptr<Material> material;
};

}  // namespace pine
