#include <pine/core/lightsampler.h>
#include <pine/core/integrator.h>
#include <pine/core/voxelizer.h>

namespace pine {

class VoxelConeIntegrator : public RayIntegrator {
public:
  VoxelConeIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler)
      : RayIntegrator{psl::move(accel), psl::move(sampler)},
        light_sampler{psl::move(light_sampler)} {
  }
  void render(Scene& scene) override;
  vec3 radiance(Scene& scene, Ray ray, Interaction it, bool is_hit, Sampler& sampler) override;

private:
  AABB aabb;
  vec3i resolution;
  float footprint;
  psl::vector<Voxels> mipmaps;
  LightSampler light_sampler;
};

}  // namespace pine
