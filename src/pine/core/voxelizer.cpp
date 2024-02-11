#include <pine/core/lightsampler.h>
#include <pine/core/voxelizer.h>
#include <pine/core/parallel.h>
#include <pine/core/profiler.h>
#include <pine/core/sampler.h>
#include <pine/core/accel.h>

namespace pine {

Array3d<Voxel> voxelize(const Scene& scene, vec3i resolution) {
  Profiler _("Voxelize");
  auto accel = EmbreeAccel();
  accel.build(&scene);
  auto lsampler = UniformLightSampler();
  lsampler.build(&scene);
  auto spp = 32;
  auto samplers = psl::vector_n_of(n_threads(), HaltonSampler(spp));

  auto voxels = Array3d<Voxel>(resolution);

  auto aabb = scene.get_aabb();
  auto epsilon = max_value(aabb.diagonal()) * 1e-5f;

  for (int axis = 0; axis < 3; axis++) {
    auto i0 = axis, i1 = (axis + 1) % 3, i2 = (axis + 2) % 3;
    auto proj_res = vec2i(resolution[i0], resolution[i1]);
    for (int si = 0; si < spp; si++)
      parallel_for(proj_res, [&](vec2i ip) {
        auto& sampler = samplers[threadIdx];
        sampler.start_pixel(ip, si);
        auto p = vec2(ip + sampler.get2d()) / proj_res;
        auto ray = Ray();
        ray.o[i0] = lerp(p[0], aabb.lower[i0], aabb.upper[i0]);
        ray.o[i1] = lerp(p[1], aabb.lower[i1], aabb.upper[i1]);
        ray.o[i2] = aabb.upper[i2] + epsilon;
        ray.d[i2] = -1;
        auto it = Interaction();
        while (accel.intersect(ray, it)) {
          auto& voxel = voxels[aabb.relative_position(it.p) * resolution];
          auto L = vec3(0);
          if (!it.material()->is_delta()) {
            if (auto ls = lsampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
              if (!accel.hit(it.spawn_ray(ls->wo, ls->distance))) {
                auto f = it.material()->F({it, it.n, it.n});
                auto cosine = absdot(ls->wo, it.n);
                L = ls->le * cosine * f / ls->pdf;
              }
            }
          }
          auto alpha = voxel.nsamples + 1.0f;
          voxel.color = lerp(1 / alpha, voxel.color, L);
          voxel.opacity = lerp(1 / alpha, voxel.opacity, 1.0f);
          voxel.nsamples++;
          ray.tmin = ray.tmax + epsilon;
          ray.tmax = float_max;
        }
      });
  }

  return voxels;
}

psl::vector<Array3d<Voxel>> build_mipmap(Array3d<Voxel> original) {
  auto size = original.size();
  CHECK_EQ(size.x, size.y);
  CHECK_EQ(size.x, size.z);
  CHECK(psl::is_power_of_2(size.x));
  auto mipmaps = psl::vector<Array3d<Voxel>>(psl::log2i(size.x) + 1);

  mipmaps[0] = psl::move(original);
  for (size_t i = 1; i < mipmaps.size(); i++) {
    const auto& prev = mipmaps[i - 1];
    auto& mipmap = mipmaps[i];
    mipmap = Array3d<Voxel>(size / (1 << i));
    for_3d(mipmap.size(), [&](vec3i p) {
      mipmap[p].opacity =
          prev[p * 2 + vec3i(0, 0, 0)].opacity + prev[p * 2 + vec3i(0, 0, 1)].opacity +
          prev[p * 2 + vec3i(0, 1, 0)].opacity + prev[p * 2 + vec3i(0, 1, 1)].opacity +
          prev[p * 2 + vec3i(1, 0, 0)].opacity + prev[p * 2 + vec3i(1, 0, 1)].opacity +
          prev[p * 2 + vec3i(1, 1, 0)].opacity + prev[p * 2 + vec3i(1, 1, 1)].opacity;
      mipmap[p].color = prev[p * 2 + vec3i(0, 0, 0)].color + prev[p * 2 + vec3i(0, 0, 1)].color +
                        prev[p * 2 + vec3i(0, 1, 0)].color + prev[p * 2 + vec3i(0, 1, 1)].color +
                        prev[p * 2 + vec3i(1, 0, 0)].color + prev[p * 2 + vec3i(1, 0, 1)].color +
                        prev[p * 2 + vec3i(1, 1, 0)].color + prev[p * 2 + vec3i(1, 1, 1)].color;
      mipmap[p].opacity /= 8;
      mipmap[p].color /= 8;
    });
  }

  return mipmaps;
}

}  // namespace pine
