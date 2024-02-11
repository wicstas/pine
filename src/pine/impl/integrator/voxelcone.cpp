#include <pine/impl/integrator/voxelcone.h>
#include <pine/core/voxelizer.h>
#include <pine/core/parallel.h>
#include <pine/core/profiler.h>

namespace pine {

void VoxelConeIntegrator::render(Scene &scene) {
  auto aabb = scene.get_aabb().extend_to_max_axis();
  auto resolution = vec3i(128);
  auto voxel_size = aabb.diagonal() / resolution;

  auto voxels = voxelize(scene, resolution);
  // auto mipmaps = build_mipmap(voxels);
  // voxels = mipmaps[0];
  scene.geometries.clear();

  auto transform = scale(voxel_size) * translate(-0.5f, -0.5f, -0.5f);
  for_3d(resolution, [&](vec3i ip) {
    auto p = (ip + vec3(0.5f)) / resolution;
    auto voxel = voxels[ip];
    if (voxel.opacity > 0.0f)
      add_box(scene, translate(aabb.absolute_position(p)) * transform,
              DiffuseMaterial(vec3(voxel.color)));
  });

  auto integ = CustomRayIntegrator(
      accel, samplers[0], [](CustomRayIntegrator &integ, Scene &, Ray ray, Sampler &) -> vec3 {
        auto it = Interaction();
        if (!integ.intersect(ray, it))
          return vec3(0.0f);
        auto mec = MaterialEvalCtx(it, it.n, it.n);
        return it.material()->F(mec) * Pi;
      });
  integ.render(scene);
}

}  // namespace pine