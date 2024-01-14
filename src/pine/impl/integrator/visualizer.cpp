#include <pine/impl/integrator/visualizer.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>
#include <pine/core/parallel.h>
#include <pine/core/fileio.h>

namespace pine {

VisualizerIntegrator::VisualizerIntegrator(Accel accel, Sampler sampler, psl::string type)
    : RayIntegrator{psl::move(accel), psl::move(sampler)} {
  if (type == "n")
    viz_type = Normal;
  else if (type == "p")
    viz_type = Position;
  else if (type == "uv")
    viz_type = UV;
  else if (type == "bvh")
    viz_type = BVH;
  else
    exception("VisualizerIntegrator doesn't recognize the enum `", type, '`');
}

vec3 VisualizerIntegrator::radiance(Scene&, Ray ray, Sampler&) {
  Interaction it;
  if (intersect(ray, it)) {
    switch (viz_type) {
      case Normal: return it.n; break;
      case Position: return it.p; break;
      case UV: return vec3{it.uv}; break;
      case BVH: return color_map(it.bvh / 10.0f); break;
    }
  }
  return vec3(0.0f);
}

}  // namespace pine