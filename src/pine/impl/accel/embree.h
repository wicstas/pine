#pragma once

#include <pine/core/defines.h>

#include <psl/vector.h>

namespace pine {

class EmbreeAccel {
public:
  EmbreeAccel() = default;
  void build(const Scene* scene);
  bool hit(Ray ray) const;
  bool intersect(Ray& ray, Interaction& it) const;

private:
  const Scene* scene;
  void* rtc_scene;
};

}  // namespace pine