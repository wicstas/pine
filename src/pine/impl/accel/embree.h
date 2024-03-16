#pragma once
#include <pine/core/interaction.h>
#include <pine/core/bbox.h>
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
  psl::opaque_shared_ptr rtc_device;
  psl::opaque_shared_ptr rtc_scene;
};

}  // namespace pine