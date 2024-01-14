#pragma once

#include <pine/impl/accel/bvh.h>

namespace pine {

struct Accel : psl::Variant<BVH> {
  using Variant::Variant;

  void build(const Scene* scene) {
    return dispatch([&](auto&& x) -> decltype(auto) { return x.build(scene); });
  }
  bool hit(Ray ray) const {
    return dispatch([&](auto&& x) -> decltype(auto) { return x.hit(ray); });
  }
  bool intersect(Ray& ray, Interaction& it) const {
    return dispatch([&](auto&& x) -> decltype(auto) { return x.intersect(ray, it); });
  }
};

}  // namespace pine
