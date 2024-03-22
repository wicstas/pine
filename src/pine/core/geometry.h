#pragma once

#include <pine/core/interaction.h>
#include <pine/core/material.h>
#include <pine/core/bbox.h>
#include <pine/core/ray.h>

#include <psl/function.h>
#include <psl/variant.h>

namespace pine {

struct Plane {
  Plane(vec3 position, vec3 normal);
  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray) const;
  void compute_surface_info(SurfaceInteraction& it) const;
  AABB get_aabb() const;
  ShapeSample sample(vec3 p, vec2 u) const;
  float pdf(const Interaction& it, const SurfaceInteraction& git, const Ray& ray) const;
  float area() const {
    return float_max;
  }

private:
  vec3 position;
  vec3 n;
  vec3 u, v;
};

struct Sphere {
  Sphere(vec3 position, float radius);
  static float compute_t(vec3 ro, vec3 rd, float tmin, vec3 p, float r);
  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray) const;
  void compute_surface_info(SurfaceInteraction& it) const;
  AABB get_aabb() const;
  ShapeSample sample(vec3, vec2 u) const;
  float pdf(const Interaction& it, const SurfaceInteraction& git, const Ray& ray) const;
  float area() const {
    return 4 * Pi * r * r;
  }

private:
  vec3 c;
  float r;
};

struct Disk {
  Disk(vec3 position, vec3 normal, float r);
  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray) const;
  void compute_surface_info(SurfaceInteraction& it) const;
  AABB get_aabb() const;
  ShapeSample sample(vec3, vec2) const;
  float pdf(const Interaction& it, const SurfaceInteraction& git, const Ray& ray) const;
  float area() const {
    return Pi * r * r;
  }

private:
  vec3 position;
  vec3 n;
  vec3 u, v;
  float r;
};

struct Line {
  Line(vec3 p0, vec3 p1, float thickness);

  bool hit(const Ray& ray) const;
  bool intersect(Ray& ray) const;
  void compute_surface_info(SurfaceInteraction& it) const;
  AABB get_aabb() const;
  ShapeSample sample(vec3, vec2) const;
  float pdf(const Interaction& it, const SurfaceInteraction& git, const Ray& ray) const;
  float area() const {
    return thickness * 2 * Pi * len;
  }

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
  bool intersect(Ray& ray) const;
  void compute_surface_info(SurfaceInteraction& it) const;
  AABB get_aabb() const;
  ShapeSample sample(vec3, vec2 u) const;
  float pdf(const Interaction& it, const SurfaceInteraction& git, const Ray& ray) const;
  float area() const {
    return lx * ly;
  }

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
  bool intersect(Ray& ray) const;
  void compute_surface_info(SurfaceInteraction& it) const;

  AABB get_aabb() const;
  ShapeSample sample(vec3, vec2 u) const;
  float pdf(const Interaction& it, const SurfaceInteraction& git, const Ray& ray) const;
  float area() const {
    return length(cross(v1 - v0, v2 - v0)) / 2;
  }

private:
  vec3 v0, v1, v2;
  vec3 n;
};

struct TriangleMesh {
  TriangleMesh() = default;
  TriangleMesh(psl::vector<vec3> vertices, psl::vector<vec3u32> indices,
               psl::vector<vec2> texcoords = {}, psl::vector<vec3> normals = {});

  bool hit(const Ray&) const {
    PINE_UNREACHABLE;
  }
  bool intersect(Ray&) const {
    PINE_UNREACHABLE;
  }
  void compute_surface_info(SurfaceInteraction&) const {
    PINE_UNREACHABLE;
  }
  AABB get_aabb() const;
  float area() const {
    PINE_UNREACHABLE;
  }
  ShapeSample sample(vec3, vec2) const {
    PINE_UNREACHABLE;
  }
  float pdf(const Interaction&, const SurfaceInteraction&, const Ray&) const {
    PINE_UNREACHABLE;
  }

  bool hit(const Ray& ray, int index) const;
  bool intersect(Ray& ray, int index) const;
  void compute_surface_info(SurfaceInteraction& it, int index) const;
  AABB get_aabb(size_t index) const;

  size_t num_triangles() const {
    return indices.size();
  }

  TriangleMesh& apply(mat4 m) {
    for (auto& v : vertices)
      v = vec3(m * vec4(v, 1.0f));
    return *this;
  }

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

  psl::vector<vec3> vertices;
  psl::vector<vec3> normals;
  psl::vector<vec2> texcoords;
  psl::vector<vec3u32> indices;
};

TriangleMesh height_map_to_mesh(const Array2d<float>& height_map);
TriangleMesh height_map_to_mesh(vec2i resolution, psl::function<float(vec2)> height_function);

struct Shape : psl::variant<AABB, OBB, Sphere, Plane, Triangle, Rect, Disk, Line, TriangleMesh> {
  using variant::variant;

  bool hit(const Ray& ray) const {
    return dispatch([&](auto&& x) { return x.hit(ray); });
  }
  bool intersect(Ray& ray) const {
    return dispatch([&](auto&& x) { return x.intersect(ray); });
  }
  void compute_surface_info(SurfaceInteraction& it) const {
    return dispatch([&](auto&& x) { return x.compute_surface_info(it); });
  }
  AABB get_aabb() const {
    return dispatch([&](auto&& x) { return x.get_aabb(); });
  }
  float area() const {
    return dispatch([&](auto&& x) { return x.area(); });
  }
  psl::optional<ShapeSample> sample(vec3 p, vec2 u) const {
    auto ss = dispatch([&](auto&& x) { return x.sample(p, u); });
    if (ss.pdf <= 0.0f)
      return psl::nullopt;
    return ss;
  }
  float pdf(const Interaction& it, const SurfaceInteraction& git, const Ray& ray) const {
    return dispatch([&](auto&& x) { return x.pdf(it, git, ray); });
  }
};

struct Geometry {
  Geometry(Shape shape, psl::shared_ptr<Material> material)
      : id(global_id++), shape{psl::move(shape)}, material{psl::move(material)} {
  }

  bool hit(const Ray& ray) const {
    return shape.hit(ray);
  }
  bool intersect(Ray& ray) const {
    return shape.intersect(ray);
  }
  void compute_surface_info(SurfaceInteraction& it) const {
    shape.compute_surface_info(it);
  }
  AABB get_aabb() const {
    return shape.get_aabb();
  }
  psl::optional<ShapeSample> sample(vec3 p, vec2 u) const {
    return shape.sample(p, u);
  }
  float pdf(const Interaction& it, const SurfaceInteraction& git, const Ray& ray) const {
    return shape.pdf(it, git, ray);
  }
  float area() const {
    return shape.area();
  }

  int id;
  Shape shape;
  psl::shared_ptr<Material> material;

private:
  static int global_id;
};

void geometry_context(Context& ctx);

}  // namespace pine
