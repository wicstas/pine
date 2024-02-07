#pragma once

#include <pine/impl/accel/embree.h>
#include <pine/impl/accel/bvh.h>

namespace pine {

struct Accel : psl::variant<BVH, EmbreeAccel> {
  using variant::variant;

  void build(const Scene* scene) {
    return dispatch([&](auto&& x) { return x.build(scene); });
  }
  bool hit(Ray ray) const {
    return dispatch([&](auto&& x) { return x.hit(ray); });
  }
  bool intersect(Ray& ray, Interaction& it) const {
    return dispatch([&](auto&& x) { return x.intersect(ray, it); });
  }
};

}  // namespace pine
