#include <pine/impl/integrator/voxelcone.h>
#include <pine/core/voxelizer.h>
#include <pine/core/parallel.h>
#include <pine/core/profiler.h>

namespace pine {

void VoxelConeIntegrator::render(Scene& scene) {
  aabb = scene.get_aabb().extend_to_max_axis();
  resolution = vec3i(128);

  auto voxels = voxelize(scene, aabb, resolution);
  footprint = aabb.diagonal()[0] / resolution[0];
  mipmaps = build_mipmap(psl::move(voxels));
  light_sampler.build(&scene);

  // scene.geometries.clear();
  // voxels = psl::move(mipmaps[0]);
  // auto transform = scale(aabb.diagonal() / voxels.size()) * translate(-0.5f, -0.5f, -0.5f);
  // parallel_for(voxels.size(), [&](vec3i ip) {
  //   auto p = (ip + vec3(0.5f)) / voxels.size();
  //   if (auto voxel = voxels.find(ip))
  //     add_box(scene, translate(aabb.absolute_position(p)) * transform,
  //             DiffuseMaterial(voxel->color));
  // });
  // CustomRayIntegrator(
  //     accel, samplers[0],
  //     [](CustomRayIntegrator&, Scene&, Ray, Interaction it, bool is_hit, Sampler&) -> vec3 {
  //       if (!is_hit)
  //         return vec3(0.0f);
  //       return it.material()->f({it, it.n, it.n}) * Pi;
  //     })
  //     .render(scene);
  // return;

  accel.build(&scene);
  auto& film = scene.camera.film();
  set_progress(0);
  Profiler _("[Integrator]Rendering");
  Atomic<int64_t> max_index = 0;
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = samplers[threadIdx];
    sampler.start_pixel(p, 0);
    auto p_film = vec2(p) / film.size();
    auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
    auto it = Interaction();
    auto is_hit = intersect(ray, it);
    auto L = radiance(ray, it, is_hit, sampler);
    scene.camera.film().add_sample(p, L);

    if (p.x == 0) {
      max_index = psl::max<int64_t>(max_index, p.x + p.y * film.size().x);
      set_progress(float(max_index) / area(film.size()));
    }
  });

  set_progress(1);
}

static vec4 interpolate(const Voxels& voxels, vec3i p, vec3 fr, vec3 d) {
  const auto half = 0.25f;
  const auto total_volume = psl::powi(half * 2, 3);
  const auto max_n = 8;
  auto n = 0;
  vec3i ps[max_n];
  float weights[max_n];
  auto f0 = fr - vec3(half);
  auto f1 = fr + vec3(half);
  for (int z = -1; z <= 1; z++)
    for (int y = -1; y <= 1; y++)
      for (int x = -1; x <= 1; x++) {
        auto dx = psl::max(psl::min(x + 1.0f, f1.x) - psl::max<float>(x, f0.x), 0.0f);
        auto dy = psl::max(psl::min(y + 1.0f, f1.y) - psl::max<float>(y, f0.y), 0.0f);
        auto dz = psl::max(psl::min(z + 1.0f, f1.z) - psl::max<float>(z, f0.z), 0.0f);
        auto overlap_volume = dx * dy * dz;
        if (overlap_volume > 0) {
          ps[n] = clamp(p + vec3i(x, y, z), vec3i(0), vec3i(voxels.size()) - vec3i(1));
          weights[n] = overlap_volume / total_volume;
          n++;
        }
      }
  auto color = vec3(0.0f);
  auto opacity = 0.0f;
  d = d * d;
  for (int i = 0; i < n; i++) {
    if (auto it = voxels.find(ps[i])) {
      auto op = weights[i] * dot(it->opacity, d);
      opacity += op;
      color += it->color * op;
    }
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

vec3 VoxelConeIntegrator::radiance(Ray ray, Interaction it, bool is_hit, Sampler& sampler) {
  if (is_hit) {
    it.n = face_same_hemisphere(it.n, -ray.d);

    auto get_voxel = [this](float level, vec3 p, vec3 d) {
      auto l0 = size_t(psl::floor(level));
      auto l1 = psl::min<size_t>(psl::ceil(level), mipmaps.size() - 1);
      auto p0 = aabb.relative_position(p) * mipmaps[l0].size();
      auto ip0 = vec3i(floor(p0));
      auto m0 = interpolate(mipmaps[l0], ip0, p0 - ip0, d);
      if (l0 == l1)
        return m0;
      auto p1 = aabb.relative_position(p) * mipmaps[l1].size();
      auto ip1 = vec3i(floor(p1));
      auto m1 = interpolate(mipmaps[l1], ip1, p1 - ip1, d);
      return lerp(psl::fract(level), m0, m1);
    };

    auto cone_trace = [&](vec3 dir, float aperture) {
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
          t = t + psl::max(r, footprint) / 1.5f;
        r = t * sinr * 2;
      }

      if (it.material()->is_delta())
        return c;
      return c * absdot(it.n, dir) * it.material()->f({it, -ray.d, dir});
    };

    auto indirect = vec3(0);
    if (it.material()->is<DiffuseMaterial>()) {
      for (int i = 0; i < cone_sample_count; i++)
        indirect +=
            cone_trace(face_same_hemisphere(cone_sample_directions[i], it.n), cone_aperture);
      indirect = 2.0f * indirect * Pi / cone_sample_count;
    } else {
      indirect += cone_trace(
          Reflect(-ray.d, it.n),
          psl::max(psl::sqr(it.material()->roughness_amount({it.p, it.n, it.uv})), Pi / 256));
      indirect = indirect * Pi;
    }

    auto direct = vec3(0);
    for (int si = 0; si < samples_per_pixel; si++) {
      if (!it.material()->is_delta()) {
        if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
          if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
            auto cosine = absdot(ls->wo, it.n);
            auto mec = MaterialEvalCtx(it, -ray.d, ls->wo);
            auto f = it.material()->f(mec);
            direct += cosine * ls->le / ls->pdf * f;
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