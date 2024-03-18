#pragma once
#include <pine/impl/accel/embree.h>
#include <pine/impl/accel/bvh.h>

#include <psl/variant.h>
#include <psl/span.h>

namespace pine {

struct Accel : psl::variant<BVH, EmbreeAccel> {
  using variant::variant;

  void build(const psl::vector<psl::shared_ptr<pine::Geometry>>* geometries) {
    return dispatch([&](auto&& x) { return x.build(geometries); });
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

}  // namespace pine
