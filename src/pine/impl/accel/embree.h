#pragma once
#include <pine/core/defines.h>

#include <psl/vector.h>
#include <psl/span.h>

namespace pine {

class EmbreeAccel {
public:
  EmbreeAccel() = default;
  void build(const Scene* scene);
  bool hit(Ray ray) const;
  uint8_t hit8(psl::span<const Ray> rays) const;
  bool intersect(Ray& ray, Interaction& it) const;

private:
  const Scene* scene;
  void* rtc_scene;
};

}  // namespace pine