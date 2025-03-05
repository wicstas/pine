#pragma once
#include <pine/core/interaction.h>
#include <pine/core/bbox.h>
#include <pine/core/ray.h>

#include <psl/function.h>
#include <psl/vector.h>
#include <psl/span.h>

namespace pine {

class EmbreeAccel {
 public:
  EmbreeAccel() = default;
  EmbreeAccel(const Scene* scene) { build(scene); }
  void build(const Scene* scene);

  bool hit(Ray ray) const;
  uint8_t hit8(psl::span<const Ray> rays) const;
  bool intersect(Ray& ray, SurfaceInteraction& it) const;

 private:
  const Scene* scene;
  psl::opaque_shared_ptr rtc_device;
  psl::opaque_shared_ptr rtc_scene;
  psl::vector<psl::opaque_shared_ptr> instancing_scenes;
  psl::vector<uint32_t> indices_to_instancing_index;
  psl::vector<uint32_t> indices_to_instance_index;
};

}  // namespace pine