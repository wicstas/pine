#include <pine/core/lightsampler.h>
#include <pine/core/voxelizer.h>
#include <pine/core/parallel.h>
#include <pine/core/profiler.h>
#include <pine/core/sampler.h>
#include <pine/core/accel.h>

namespace pine {

Voxels voxelize(const Scene& scene, AABB aabb, vec3i resolution) {
  Profiler _("Voxelization");
  Debugr("Voxelizing scene");
  auto accel = EmbreeAccel();
  accel.build(&scene);
  auto lsampler = UniformLightSampler();
  lsampler.build(&scene);
  auto spp = 8;
  auto samplers = psl::vector_n_of(n_threads(), HaltonSampler(spp));

  auto voxels = Voxels(resolution);
  auto epsilon = max_value(aabb.diagonal()) * 1e-5f;
  auto lock = SpinLock();

  for (int axis = 0; axis < 3; axis++) {
    auto i0 = axis, i1 = (axis + 1) % 3, i2 = (axis + 2) % 3;
    auto proj_res = vec2i(resolution[i0], resolution[i1]);
    for (int si = 0; si < spp; si++)
      parallel_for(proj_res, [&](vec2i ip) {
        auto& sampler = samplers[threadIdx];
        sampler.start_pixel(ip, si);
        auto p = vec2(ip + sampler.get2d()) / proj_res;
        auto ray = Ray();
        ray.o[i0] = psl::lerp(p[0], aabb.lower[i0], aabb.upper[i0]);
        ray.o[i1] = psl::lerp(p[1], aabb.lower[i1], aabb.upper[i1]);
        ray.o[i2] = aabb.upper[i2] + epsilon;
        ray.d[i2] = -1;
        auto it = Interaction();
        while (accel.intersect(ray, it)) {
          it.n = face_same_hemisphere(it.n, -ray.d);
          auto ip = vec3i(floor(aabb.relative_position(it.p) * resolution));
          ip = clamp(ip, vec3i(0), vec3i(voxels.size()) - vec3i(1));
          auto L = vec3(0);
          if (it.material()->is<DiffuseMaterial>()) {
            if (auto ls = lsampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
              if (!accel.hit(it.spawn_ray(ls->wo, ls->distance))) {
                auto f = it.material()->f({it, it.n, it.n});
                auto cosine = absdot(ls->wo, it.n);
                L += ls->le * cosine * f / ls->pdf;
              }
            }
          }
          lock.lock();
          auto& voxel = voxels[ip];
          auto alpha = voxel.nsamples + 1.0f;
          voxel.color = lerp(1 / alpha, voxel.color, L);
          voxel.opacity = vec3(1, 1, 1);
          voxel.nsamples++;
          lock.unlock();
          ray.tmin = ray.tmax + epsilon;
          ray.tmax = float_max;
        }
      });
  }

  Debug(" done.");

  return voxels;
}

psl::vector<Voxels> build_mipmap(Voxels original) {
  Profiler _("Build mipmap");
  Debugr("Building mipmap");
  auto size = original.size();
  CHECK_EQ(size.x, size.y);
  CHECK_EQ(size.x, size.z);
  CHECK(psl::is_power_of_2(size.x));
  auto mipmaps = psl::vector<Voxels>(psl::log2i(size.x) + 1);
  auto lock = SpinLock();

  mipmaps[0] = psl::move(original);
  for (size_t i = 1; i < mipmaps.size(); i++) {
    const auto& prev = mipmaps[i - 1];
    auto& mipmap = mipmaps[i];
    mipmap = Voxels(size / (1 << i));
    parallel_for(mipmap.size(), [&](vec3i p) {
      auto v = Voxel();
      Voxel vs[2][2][2];
      for_3d({2, 2, 2}, [&](vec3i i) {
        if (auto it = prev.find(p * 2 + i))
          vs[i[2]][i[1]][i[0]] = *it;
      });
      v.opacity[0] = (psl::max(vs[0][0][0].opacity[0], vs[0][0][1].opacity[0]) +
                      psl::max(vs[0][1][0].opacity[0], vs[0][1][1].opacity[0]) +
                      psl::max(vs[1][0][0].opacity[0], vs[1][0][1].opacity[0]) +
                      psl::max(vs[1][1][0].opacity[0], vs[1][1][1].opacity[0])) /
                     4;
      v.opacity[1] = (psl::max(vs[0][0][0].opacity[1], vs[0][1][0].opacity[1]) +
                      psl::max(vs[0][0][1].opacity[1], vs[0][1][1].opacity[1]) +
                      psl::max(vs[1][0][0].opacity[1], vs[1][1][0].opacity[1]) +
                      psl::max(vs[1][0][1].opacity[1], vs[1][1][1].opacity[1])) /
                     4;
      v.opacity[2] = (psl::max(vs[0][0][0].opacity[2], vs[1][0][0].opacity[2]) +
                      psl::max(vs[0][0][1].opacity[2], vs[1][0][1].opacity[2]) +
                      psl::max(vs[0][1][0].opacity[2], vs[1][1][0].opacity[2]) +
                      psl::max(vs[0][1][1].opacity[2], vs[1][1][1].opacity[2])) /
                     4;
      auto opacity_sum = 0.0f;
      for_3d({2, 2, 2}, [&](vec3i i) { opacity_sum += sum(vs[i[2]][i[1]][i[0]].opacity); });
      if (opacity_sum == 0)
        return;
      for_3d({2, 2, 2}, [&](vec3i i) {
        v.color += vs[i[2]][i[1]][i[0]].color * sum(vs[i[2]][i[1]][i[0]].opacity) / opacity_sum;
      });
      lock.lock();
      mipmap[p] = v;
      lock.unlock();
    });
  }

  Debug(" done.");
  return mipmaps;
}

}  // namespace pine
