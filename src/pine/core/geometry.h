#pragma once

#include <pine/core/bbox.h>
#include <pine/core/interaction.h>
#include <pine/core/material.h>
#include <pine/core/ray.h>
#include <psl/function.h>
#include <psl/variant.h>

namespace pine {

struct Plane {
  Plane(vec3 position, vec3 normal);
  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  void compute_surface_info(vec3 p, SurfaceInteraction& it) const;
  AABB get_aabb() const;
  psl::optional<ShapeSample> sample(vec3 p, vec2 u) const;
  float pdf(const Ray& ray, vec3 ps, vec3 ns) const;
  float area() const { return float_max; }

 private:
  vec3 position;
  vec3 n;
  vec3 u, v;
};

struct Sphere {
  Sphere(vec3 position, float radius);
  static float compute_t(vec3 ro, vec3 rd, float tmin, vec3 p, float r);
  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  void compute_surface_info(vec3 p, SurfaceInteraction& it) const;
  AABB get_aabb() const;
  psl::optional<ShapeSample> sample(vec3, vec2 u) const;
  float pdf(const Ray& ray, vec3 ps, vec3 ns) const;
  float area() const { return 4 * Pi * r * r; }

 private:
  vec3 c;
  float r;
};

struct Disk {
  Disk(vec3 position, vec3 normal, float r);
  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  void compute_surface_info(vec3 p, SurfaceInteraction& it) const;
  AABB get_aabb() const;
  psl::optional<ShapeSample> sample(vec3, vec2) const;
  float pdf(const Ray& ray, vec3 ps, vec3 ns) const;
  float area() const { return Pi * r * r; }

 private:
  vec3 position;
  vec3 n;
  vec3 u, v;
  float r;
};

struct Line {
  Line(vec3 p0, vec3 p1, float thickness);

  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  void compute_surface_info(vec3 p, SurfaceInteraction& it) const;
  AABB get_aabb() const;
  psl::optional<ShapeSample> sample(vec3, vec2) const;
  float pdf(const Ray& ray, vec3 ps, vec3 ns) const;
  float area() const { return thickness * 2 * Pi * len; }

 private:
  vec3 p0, p1;
  mat3 tbn;
  float thickness;
  float len;
};

struct Rect {
  Rect(vec3 position, vec3 ex, vec3 ey, bool flip_normal = false);
  static Rect from_vertex(vec3 v0, vec3 v1, vec3 v2);
  Rect& apply(mat4 m);

  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  void compute_surface_info(vec3 p, SurfaceInteraction& it) const;
  AABB get_aabb() const;
  psl::optional<ShapeSample> sample(vec3, vec2 u) const;
  float pdf(const Ray& ray, vec3 ps, vec3 ns) const;
  float area() const { return lx * ly; }

 private:
  vec3 position, ex, ey, n;
  float lx, ly;
  vec3 rx, ry;
};

struct Triangle {
  Triangle() = default;
  Triangle(vec3 v0, vec3 v1, vec3 v2, vec3 n);
  Triangle(vec3 v0, vec3 v1, vec3 v2);

  static bool hit(const Ray& ray, vec3 v0, vec3 v1, vec3 v2);
  static bool intersect(Ray& ray, vec3 v0, vec3 v1, vec3 v2);
  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  void compute_surface_info(vec3 p, SurfaceInteraction& it) const;

  AABB get_aabb() const;
  psl::optional<ShapeSample> sample(vec3, vec2 u) const;
  float pdf(const Ray& ray, vec3 ps, vec3 ns) const;
  float area() const { return length(cross(v1 - v0, v2 - v0)) / 2; }

 private:
  vec3 v0, v1, v2;
  vec3 n;
};

struct Cone {
  Cone(vec3 p, vec3 n, float r, float h);

  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  void compute_surface_info(vec3 p, SurfaceInteraction& it) const;

  AABB get_aabb() const { return bottom.get_aabb().extend(p); }
  psl::optional<ShapeSample> sample(vec3, vec2 u) const;
  float pdf(const Ray& ray, vec3 ps, vec3 ns) const;
  float area() const { return psl::sqrt(r * r + h * h) * Pi * r + bottom.area(); }

 private:
  Disk bottom;
  vec3 p;
  vec3 n;
  float r;
  float h;
  float A, A2, S;
};
struct Cylinder {
  Cylinder(vec3 p0, vec3 p1, float r)
      : p0(p0), p1(p1), n(normalize(p1 - p0)), r(r), bottom(p0, -n, r), top(p0, n, r) {}

  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  void compute_surface_info(vec3 p, SurfaceInteraction& it) const;

  AABB get_aabb() const { return union_(bottom.get_aabb(), top.get_aabb()); }
  psl::optional<ShapeSample> sample(vec3, vec2) const { PINE_UNREACHABLE; }
  float pdf(const Ray&, vec3, vec3) const { PINE_UNREACHABLE; }
  float area() const { PINE_UNREACHABLE; }

 private:
  vec3 p0, p1, n;
  float r;
  Disk bottom, top;
};
struct Mesh {
  Mesh() = default;
  Mesh(psl::vector<vec3> vertices, psl::vector<vec3u32> indices, psl::vector<vec2> texcoords = {},
       psl::vector<vec3> normals = {});

  bool hit(const Ray&) const { PINE_UNREACHABLE; }
  bool intersect(Ray&) const { PINE_UNREACHABLE; }
  void compute_surface_info(vec3, SurfaceInteraction&) const { PINE_UNREACHABLE; }
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  AABB get_aabb() const;
  float area() const {  // TODO
    return get_triangle(0).area() * num_triangles();
  }
  psl::optional<ShapeSample> sample(vec3 p, vec2 u, float u1) const {
    if (num_triangles() == 0) return psl::nullopt;

    auto ss = get_triangle(num_triangles() * u1).sample(p, u);
    ss->pdf /= num_triangles();

    return ss;
  }
  float pdf(const Ray& ray, vec3, vec3 ns) const {
    return psl::sqr(ray.tmax) / (area() * absdot(ns, ray.d));
  }

  bool hit(const Ray& ray, int index) const;
  bool intersect(Ray& ray, int index) const;
  void compute_surface_info(vec3 p, SurfaceInteraction& it, int index) const;
  AABB get_aabb(size_t index) const;

  size_t num_triangles() const { return indices.size(); }
  Triangle get_triangle(int index) const {
    auto face = indices[index];
    return Triangle(vertices[face[0]], vertices[face[1]], vertices[face[2]]);
  }

  Mesh& apply(mat4 m);

  vec3 position_of(vec3u32 face, vec2 uv) const {
    return lerp(uv[0], uv[1], vertices[face[0]], vertices[face[1]], vertices[face[2]]);
  }
  psl::optional<vec3> normal_of(vec3u32 face, vec2 uv) const {
    if (normals.size())
      return normalize(lerp(uv[0], uv[1], normals[face[0]], normals[face[1]], normals[face[2]]));
    else
      return psl::nullopt;
  }
  psl::optional<vec2> texcoord_of(vec3u32 face, vec2 uv) const {
    if (texcoords.size())
      return lerp(uv[0], uv[1], texcoords[face[0]], texcoords[face[1]], texcoords[face[2]]);
    else
      return psl::nullopt;
  }

  void assign_normals_and_texcoords();
  void merge(const Mesh& mesh);

  psl::vector<vec3> vertices;
  psl::vector<vec3> normals;
  psl::vector<vec2> texcoords;
  psl::vector<vec3u32> indices;
  mutable ShapeAccel* accel = nullptr;
};

Mesh heightmap(const Array2d<float>& height_map);
Mesh heightmap(vec2i resolution, psl::function<float(vec2i)> height_function);
Mesh heightmap(vec2i resolution, psl::function<float(vec2)> height_function);

struct SDF {
  SDF(vec3 center, vec3 half_size, psl::function<float(vec3)> sdf)
      : aabb(center - half_size, center + half_size), sdf(MOVE(sdf)) {
    threshold = min_value(aabb.diagonal()) * 1e-6f;
  }
  SDF(AABB aabb, psl::function<float(vec3)> sdf) : aabb(aabb), sdf(MOVE(sdf)) {
    threshold = min_value(aabb.diagonal()) * 1e-6f;
  }

  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  void compute_surface_info(vec3 p, SurfaceInteraction& it) const;

  AABB get_aabb() const { return aabb; }
  psl::optional<ShapeSample> sample(vec3, vec2) const { PINE_UNREACHABLE; }
  float pdf(const Ray&, vec3, vec3) const { PINE_UNREACHABLE; }
  float area() const { PINE_UNREACHABLE; }

 private:
  AABB aabb;
  psl::function<float(vec3)> sdf;
  float threshold;
};

struct CSGUnion {
  CSGUnion(Shape a, Shape b);

  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  void compute_surface_info(vec3, SurfaceInteraction&) const {}

  AABB get_aabb() const;
  psl::optional<ShapeSample> sample(vec3, vec2) const { PINE_UNREACHABLE; }
  float pdf(const Ray&, vec3, vec3) const { PINE_UNREACHABLE; }
  float area() const { PINE_UNREACHABLE; }

 private:
  psl::shared_ptr<Shape> a, b;
};

struct CSGIntersection {
  CSGIntersection(Shape a, Shape b);

  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  void compute_surface_info(vec3, SurfaceInteraction&) const {}

  AABB get_aabb() const;
  psl::optional<ShapeSample> sample(vec3, vec2) const { PINE_UNREACHABLE; }
  float pdf(const Ray&, vec3, vec3) const { PINE_UNREACHABLE; }
  float area() const { PINE_UNREACHABLE; }

 private:
  psl::shared_ptr<Shape> a, b;
};

struct CSGDifference {
  CSGDifference(Shape a, Shape b);

  bool hit(Ray ray) const;
  bool intersect(Ray& ray, SurfaceInteraction& it) const;
  void compute_surface_info(vec3, SurfaceInteraction&) const {}

  AABB get_aabb() const;
  psl::optional<ShapeSample> sample(vec3, vec2) const { PINE_UNREACHABLE; }
  float pdf(const Ray&, vec3, vec3) const { PINE_UNREACHABLE; }
  float area() const { PINE_UNREACHABLE; }

 private:
  psl::shared_ptr<Shape> a, b;
};

struct Shape : psl::variant<AABB, OBB, Sphere, Plane, Triangle, Cone, Cylinder, Rect, Disk, Line,
                            Mesh, SDF, CSGUnion, CSGIntersection, CSGDifference> {
  using variant::variant;

  bool hit(const Ray& ray) const {
    return dispatch([&](auto&& x) { return x.hit(ray); });
  }
  bool intersect(Ray& ray, SurfaceInteraction& it) const {
    return dispatch([&](auto&& x) { return x.intersect(ray, it); });
  }
  bool intersect_full(Ray& ray, SurfaceInteraction& it) const {
    return dispatch([&](auto&& x) {
      if (x.intersect(ray, it)) {
        x.compute_surface_info(ray(), it);
        return true;
      } else {
        return false;
      }
    });
  }
  void compute_surface_info(vec3 p, SurfaceInteraction& it) const {
    return dispatch([&](auto&& x) { return x.compute_surface_info(p, it); });
  }

  AABB get_aabb() const {
    return dispatch([&](auto&& x) { return x.get_aabb(); });
  }
  float area() const {
    return dispatch([&](auto&& x) { return x.area(); });
  }
  psl::optional<ShapeSample> sample(vec3 p, vec2 u, float u1) const {
    auto gs = dispatch([&]<typename T>(const T& x) {
      if constexpr (psl::same_as<T, Mesh>)
        return x.sample(p, u, u1);
      else
        return x.sample(p, u);
    });
    if (!gs || gs->pdf <= 0 || psl::isinf(gs->pdf)) return psl::nullopt;
    return gs;
  }
  ShapeSample sample(vec2 u, float u1) const {
    return *dispatch([&]<typename T>(const T& x) {
      if constexpr (psl::same_as<T, Mesh>)
        return x.sample({}, u, u1);
      else
        return x.sample({}, u);
    });
  }
  float pdf(const Ray& ray, vec3 ps, vec3 ns) const {
    return dispatch([&](auto&& x) { return x.pdf(ray, ps, ns); });
  }
};

struct Geometry {
  Geometry(Shape shape, psl::shared_ptr<Material> material)
      : shape{MOVE(shape)}, material{MOVE(material)} {}

  bool hit(const Ray& ray) const { return shape.hit(ray); }
  bool intersect(Ray& ray, SurfaceInteraction& it) const { return shape.intersect(ray, it); }
  void compute_surface_info(vec3 p, SurfaceInteraction& it) const {
    shape.compute_surface_info(p, it);
  }
  AABB get_aabb() const { return shape.get_aabb(); }
  psl::optional<ShapeSample> sample(vec3 p, vec2 u, float u1) const {
    return shape.sample(p, u, u1);
  }
  ShapeSurfaceSample sample(vec2 u, float u1) const { return shape.sample(u, u1); }
  float pdf(const Ray& ray, vec3 ps, vec3 ns) const { return shape.pdf(ray, ps, ns); }
  float area() const { return shape.area(); }

  Shape shape;
  psl::shared_ptr<Material> material;
};

struct InstancedShape {
  InstancedShape(Mesh shape) : shape(MOVE(shape)) {}

  InstancedShape& add(mat4 transform, Material material) {
    instances.emplace_back(transform, psl::make_shared<Material>(MOVE(material)));
    return *this;
  }
  InstancedShape& add(mat4 transform, psl::shared_ptr<Material> material) {
    instances.emplace_back(transform, MOVE(material));
    return *this;
  }

  struct Instance {
    mat4 transform;
    psl::shared_ptr<Material> material;
  };
  Shape shape;
  psl::vector<Instance> instances;
};

void geometry_context(Context& ctx);

}  // namespace pine
