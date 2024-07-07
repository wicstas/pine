#pragma once
#include <pine/impl/accel/embree.h>
#include <pine/impl/accel/bvh.h>

#include <psl/variant.h>
#include <psl/span.h>

namespace pine {

struct Accel : psl::variant<EmbreeAccel, BVH> {
  using variant::variant;

  void build(const Scene* scene) {
    return dispatch([&](auto&& x) { return x.build(scene); });
  }
  bool hit(Ray ray) const {
    return dispatch([&](auto&& x) { return x.hit(ray); });
  }
  uint8_t hit8(psl::span<const Ray> rays) const {
    return dispatch([&](auto&& x) { return x.hit8(rays); });
  }
  bool intersect(Ray& ray, SurfaceInteraction& it) const {
    return dispatch([&](auto&& x) { return x.intersect(ray, it); });
  }
};

struct ShapeAccel : psl::variant<ShapeBVH> {
  using variant::variant;

  void build(const Mesh* mesh) {
    return dispatch([&](auto&& x) { return x.build(mesh); });
  }
  bool hit(Ray ray) const {
    return dispatch([&](auto&& x) { return x.hit(ray); });
  }
  bool intersect(Ray& ray, SurfaceInteraction& it) const {
    return dispatch([&](auto&& x) { return x.intersect(ray, it); });
  }
};

}  // namespace pine
