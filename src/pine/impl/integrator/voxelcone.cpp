#include <pine/impl/integrator/voxelcone.h>
#include <pine/core/voxelizer.h>
#include <pine/core/parallel.h>
#include <pine/core/profiler.h>

namespace pine {

void VoxelConeIntegrator::render(Scene& scene) {
  aabb = scene.get_aabb().extend_to_max_axis();
  resolution = vec3i(512);

  auto voxels = voxelize(scene, aabb, resolution);
  footprint = aabb.diagonal()[0] / resolution[0];
  mipmaps = build_mipmap(voxels);
  light_sampler.build(&scene);

  // scene.geometries.clear();
  // auto transform [[maybe_unused]] = scale(vec3(footprint)) * translate(-0.5f, -0.5f, -0.5f);
  // for_3d(voxels.size(), [&](vec3i ip) {
  //   auto p = (ip + vec3(0.5f)) / voxels.size();
  //   auto voxel = voxels[ip];
  //   if (sum(voxel.opacity) > 0.0f)
  //     scene.add_geometry(Sphere(aabb.absolute_position(p), footprint / 2 / 1.5f),
  //                        DiffuseMaterial(vec3(voxel.color)));
  //   // if (sum(voxel.opacity) > 0)
  //   // add_box(scene, translate(aabb.absolute_position(p)) * transform,
  //   // DiffuseMaterial(vec3(voxel.color)));
  // });

  // CustomRayIntegrator(
  //     accel, samplers[0],
  //     [](CustomRayIntegrator&, Scene&, Ray, Interaction it, bool is_hit, Sampler&) -> vec3 {
  //       if (!is_hit)
  //         return vec3(0.0f);
  //       auto mec = MaterialEvalCtx(it, it.n, it.n);
  //       return it.material()->F(mec) * Pi;
  //     })
  //     .render(scene);
  // return;

  accel.build(&scene);
  auto& film = scene.camera.film();
  film.clear();
  set_progress(0);

  Profiler _("Rendering");
  Atomic<int64_t> max_index = 0;
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = samplers[threadIdx];
    sampler.start_pixel(p, 0);
    auto p_film = vec2(p) / scene.camera.film().size();
    auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
    auto it = Interaction();
    auto is_hit = intersect(ray, it);
    auto L = radiance(scene, ray, it, is_hit, sampler);
    scene.camera.film().add_sample(p, L);

    if (p.x == 0) {
      max_index = psl::max<int64_t>(max_index, p.x + p.y * film.size().x);
      set_progress(float(max_index) / area(film.size()));
    }
  });

  set_progress(1);
}

vec4 interpolate(const Array3d<Voxel>& voxels, vec3i p, vec3 fr, vec3 d) {
  auto v = voxels[clamp(p, vec3i(0), vec3i(voxels.size() - vec3i(1)))];
  return {v.color, dot(v.opacity, d * d)};
  const auto n = 8;
  vec3i ps[n];
  float weights[n];
  for (int i = 0; i < 3; i++) {
    if (fr[i] < 0.5f)
      p[i] -= 1;
    else
      fr[i] -= 1.0f;
  }
  for (int z = 0; z < 2; z++)
    for (int y = 0; y < 2; y++)
      for (int x = 0; x < 2; x++) {
        auto ip = p + vec3i(x, y, z);
        ps[x + y * 2 + z * 4] = clamp(ip, vec3i(0), vec3i(voxels.size() - vec3i(1)));
        auto offset = vec3(x, y, z) - vec3(0.5f);
        weights[x + y * 2 + z * 4] = volume(abs(fr + offset));
      }
  auto color = vec3(0.0f);
  auto opacity = 0.0f;
  d = d * d;
  for (int i = 0; i < n; i++) {
    const auto& voxel = voxels[ps[i]];
    auto op = weights[i] * dot(voxel.opacity, d);
    opacity += op;
    color += voxel.color * op;
  }
  if (opacity != 0.0f)
    color /= opacity;
  return vec4(color, opacity);
}

static const int cone_sample_count = 16;
static const float cone_aperture = 0.872665f;
static const vec3 cone_sample_directions[cone_sample_count]{
    vec3(0.57735, 0.57735, 0.57735),       vec3(0.57735, -0.57735, -0.57735),
    vec3(-0.57735, 0.57735, -0.57735),     vec3(-0.57735, -0.57735, 0.57735),
    vec3(-0.903007, -0.182696, -0.388844), vec3(-0.903007, 0.182696, 0.388844),
    vec3(0.903007, -0.182696, 0.388844),   vec3(0.903007, 0.182696, -0.388844),
    vec3(-0.388844, -0.903007, -0.182696), vec3(0.388844, -0.903007, 0.182696),
    vec3(0.388844, 0.903007, -0.182696),   vec3(-0.388844, 0.903007, 0.182696),
    vec3(-0.182696, -0.388844, -0.903007), vec3(0.182696, 0.388844, -0.903007),
    vec3(-0.182696, 0.388844, 0.903007),   vec3(0.182696, -0.388844, 0.903007)};

auto smooth_step(float step, auto v0, auto v1) {
  return lerp(3 * step * step - 2 * step * step * step, v0, v1);
}

vec3 VoxelConeIntegrator::radiance(Scene& scene, Ray ray, Interaction it, bool is_hit,
                                   Sampler& sampler) {
  if (is_hit) {
    it.n = face_same_hemisphere(it.n, -ray.d);

    auto get_voxel = [this](float level, vec3 p, vec3 d) {
      auto l0 = size_t(psl::floor(level));
      auto l1 = psl::clamp<size_t>(psl::ceil(level), 0, mipmaps.size() - 1);
      auto fr = psl::fract(level);
      auto p0 = aabb.relative_position(p) * mipmaps[l0].size();
      auto ip0 = vec3i(floor(p0));
      auto m0 = interpolate(mipmaps[l0], ip0, p0 - ip0, d);
      if (fr == 0.0f || l0 == l1)
        return m0;
      auto p1 = aabb.relative_position(p) * mipmaps[l1].size();
      auto ip1 = vec3i(floor(p1));
      auto m1 = interpolate(mipmaps[l1], ip1, p1 - ip1, d);
      return lerp(fr, m0, m1);
    };

    auto cone_trace = [&](vec3 dir, float aperture) {
      dir = normalize(dir);
      auto sinr = psl::sin(aperture / 2);

      auto t = footprint;
      auto r = t * sinr * 2;
      auto c = vec3(0);
      auto a = 0.0f;
      auto org = it.p + footprint * it.n;
      while (true) {
        auto mipmap_level = psl::max(psl::log2(r / footprint), 0.0f);
        if (mipmap_level >= float(mipmaps.size()))
          break;
        auto p = org + dir * t;
        if (!aabb.contains(p))
          break;

        auto voxel = get_voxel(mipmap_level, p, dir);
        auto color = vec3(voxel);
        auto opacity = voxel.w;
        c = c + (1 - a) * opacity * color;
        a = a + (1 - a) * opacity;
        if (a >= 0.98f)
          break;

        if (it.material()->is_delta())
          t += footprint / 3.0f;
        else
          t = t + psl::max(r, footprint);
        r = t * sinr * 2;
      }

      if (it.material()->is_delta())
        return c;
      return c * absdot(it.n, dir) * it.material()->F({it, -ray.d, dir});
    };

    auto indirect = vec3(0);
    if (!it.material()->is_delta()) {
      for (int i = 0; i < cone_sample_count; i++)
        indirect +=
            cone_trace(face_same_hemisphere(cone_sample_directions[i], it.n), cone_aperture);
      indirect = 2.5f * indirect * Pi / cone_sample_count;
    } else {
      indirect += cone_trace(Reflect(-ray.d, it.n), Pi / 256);
      indirect = indirect * Pi;
    }

    auto direct = vec3(0);
    for (int si = 0; si < samples_per_pixel; si++) {
      if (!it.material()->is_delta()) {
        if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
          if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
            auto cosine = absdot(ls->wo, it.n);
            auto mec = MaterialEvalCtx(it, -ray.d, ls->wo);
            auto f = it.material()->F(mec);
            direct += cosine * ls->le / ls->pdf * f;
          }
        }
        if (scene.env_light)
          if (auto ls = scene.env_light->sample(it.n, sampler.get2d())) {
            if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
              auto cosine = absdot(ls->wo, it.n);
              auto mec = MaterialEvalCtx(it, -ray.d, ls->wo);
              auto f = it.material()->F(mec);
              auto bsdf_pdf = it.material()->pdf(mec);
              direct += cosine * ls->le / ls->pdf * f * power_heuristic(ls->pdf, bsdf_pdf);
            }
          }
      }
      sampler.start_next_sample();
    }
    direct /= samples_per_pixel;

    return direct + indirect;
  } else {
    return vec3(0);
  }
}

}  // namespace pine