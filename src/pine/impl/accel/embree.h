#pragma once
#include <pine/core/interaction.h>
#include <pine/core/aabb.h>
#include <pine/core/ray.h>

#include <psl/function.h>
#include <psl/span.h>

namespace pine {

class EmbreeAccel {
public:
  EmbreeAccel() = default;
  void build(const Scene* scene);

  bool hit(Ray ray) const;
  uint8_t hit8(psl::span<const Ray> rays) const;
  bool intersect(Ray& ray, SurfaceInteraction& it) const;

private:
  const Scene* scene;
  psl::shared_ptr_with_custom_deleter<psl::Empty> rtc_device;
  psl::shared_ptr_with_custom_deleter<psl::Empty> rtc_scene;
};

}  // namespace pine