#include <pine/core/blackbody.h>
#include <pine/core/geometry.h>
#include <pine/core/sampling.h>
#include <pine/core/parallel.h>
#include <pine/core/context.h>
#include <pine/core/sampler.h>
#include <pine/core/medium.h>
#include <pine/core/scene.h>

#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/IO.h>
#include "medium.h"

namespace pine {

HomogeneousMedium::~HomogeneousMedium() = default;

HomogeneousMedium::HomogeneousMedium(Shape shape, PhaseFunction pf, vec3 sigma_a, vec3 sigma_s)
    : pf(pf), sigma_s(sigma_s), sigma_z(sigma_a + sigma_s) {
  auto material = psl::make_shared<Material>(DiffuseMaterial(vec3(1.0f)));
  scene = psl::make_shared<Scene>();
  scene->geometries.push_back(psl::make_shared<Geometry>(shape, material));
  aabb = shape.get_aabb();
  max_dim = max_value(aabb.diagonal());
  accel = EmbreeAccel();
  accel.build(scene.get());
}
psl::optional<MediumSample> HomogeneousMedium::sample(const Ray& ray, Sampler& sampler) const {
  auto inside = false;
  auto it = SurfaceInteraction();
  {
    auto r = Ray(ray.o, -ray.d, 0.0f, float_max);
    if (accel.intersect(r, it) && dot(r.d, it.n) > 0.0f)
      inside = true;
  }

  auto r = ray;
  auto sigma = average(sigma_z);
  auto t_sampled = -psl::log(1 - sampler.randf()) / sigma;
  auto t = 0.0f;

  while (true) {
    auto hit = accel.intersect(r, it);
    if (inside) {
      auto dt = r.tmax - r.tmin;
      if (t + dt > t_sampled) {
        auto W = sigma_s * exp(-(sigma_z - vec3(sigma)) * t_sampled);
        auto t_world = r.tmin + t_sampled - t;
        return MediumSample(ray(t_world), W, 1.0f, pf);
      }
      t += dt;
    }
    if (!hit)
      return psl::nullopt;

    inside = dot(r.d, it.n) < 0;
    r.tmin = r.tmax * 1.001f + max_dim * 1e-4f;
    r.tmax = ray.tmax;
  }
}
MediumPoint HomogeneousMedium::at(vec3) const {
  return {.sigma_s = sigma_s};
}
vec3 HomogeneousMedium::transmittance(vec3 p, vec3 d, float tmax, Sampler&) const {
  auto inside = false;
  auto it = SurfaceInteraction();
  {
    auto ray = Ray(p, -d, 0.0f, float_max);
    if (accel.intersect(ray, it) && dot(d, it.n) < 0.0f)
      inside = true;
  }

  auto tr = vec3(1.0f);

  auto ray = Ray(p, d, 0.0f, tmax);
  while (true) {
    auto hit = accel.intersect(ray, it);
    if (inside)
      tr *= exp(-sigma_z * (ray.tmax - ray.tmin));
    if (!hit)
      return tr;

    inside = dot(d, it.n) < 0;
    ray.tmin = ray.tmax * 1.001f + max_dim * 1e-4f;
    ray.tmax = tmax;
  }
}

struct Grid {
  Grid() = default;
  Grid(vec3i resolution, nanovdb::FloatGrid* grid) : majorants(resolution) {
    auto acc = grid->getAccessor();
    auto mi = grid->indexBBox().min();
    auto ma = grid->indexBBox().max();
    for (int i = 0; i < 3; i++) {
      min[i] = mi[i];
      max[i] = ma[i];
    }
    auto size = max - min;
    resolution_to_size = resolution / vec3(size);
    for_3d(min, max, [&](vec3 p) {
      auto& maj = majorants[(p - min) * resolution_to_size];
      maj = psl::max(maj, acc(p.x, p.y, p.z));
    });
  }

  struct Cell {
    float maj;
    float tmax;
  };
  psl::optional<Cell> get_cell(vec3 p, vec3 d, float tmin) const {
    p = p + d * tmin;
    auto idx = (p - min) * resolution_to_size;
    if (idx.x < 0 || idx.y < 0 || idx.z < 0 || idx.x >= majorants.size().x ||
        idx.y >= majorants.size().y || idx.z >= majorants.size().z)
      return psl::nullopt;
    auto maj = majorants[idx];

    auto t_x = d.x == 0.0f ? 0.0f
                           : (d.x > 0 ? psl::ceil(idx.x) - idx.x : psl::floor(idx.x) - idx.x) /
                                 resolution_to_size.x / d.x;
    auto t_y = d.y == 0.0f ? 0.0f
                           : (d.y > 0 ? psl::ceil(idx.y) - idx.y : psl::floor(idx.y) - idx.y) /
                                 resolution_to_size.y / d.y;
    auto t_z = d.z == 0.0f ? 0.0f
                           : (d.z > 0 ? psl::ceil(idx.z) - idx.z : psl::floor(idx.z) - idx.z) /
                                 resolution_to_size.z / d.z;
    auto t = psl::min(t_x, t_y, t_z);
    return Cell{maj, tmin + t};
  }

private:
  vec3 min, max, resolution_to_size;
  Array3d<float> majorants;
};

auto get_grid_info(nanovdb::FloatGrid* grid) {
  auto aabb = AABB();
  auto index_start = vec3(), index_end = vec3();
  for (int i = 0; i < 3; i++) {
    aabb.lower[i] = grid->worldBBox().min()[i];
    aabb.upper[i] = grid->worldBBox().max()[i];
    index_start[i] = grid->indexBBox().min()[i];
    index_end[i] = grid->indexBBox().max()[i];
  }
  return psl::make_tuple(
      aabb, inverse(translate(aabb.lower) * scale(aabb.diagonal()) *
                    scale(1.0f / (index_end - index_start)) * translate(-index_start)));
}

static Grid g;

VDBMedium::VDBMedium(psl::string filename, mat4 transform, PhaseFunction pf, vec3 sigma_a,
                     vec3 sigma_s, float blackbody_intensity, float temperature_scale)
    : pf(pf),
      sigma_a(sigma_a),
      sigma_s(sigma_s),
      sigma_z(sigma_a + sigma_s),
      blackbody_intensity(blackbody_intensity),
      temperature_scale(temperature_scale) {
  using Handle = nanovdb::GridHandle<nanovdb::HostBuffer>;
  {
    auto handle = nanovdb::io::readGrid(filename.c_str(), "density");
    auto grid = handle.grid<float>();

    g = Grid(vec3i(32, 32, 32), grid);

    if (!grid)
      SEVERE("[VDBMedium]`", filename, " `has no density attribute");
    sigma_a_ = average(sigma_a);
    sigma_s_ = average(sigma_s);
    sigma_z_ = sigma_a_ + sigma_s_;
    sigma_maj = sigma_z_ * grid->tree().root().getMax();
    sigma_maj_inv = 1.0f / sigma_maj;
    auto [aabb, mat] = get_grid_info(grid);
    bbox = OBB(aabb, transform);
    world2index = mat * inverse(transform);
    density_grid = grid;
    density_handle = psl::make_opaque_shared_ptr<Handle>(MOVE(handle));
  }
  if (nanovdb::io::hasGrid(filename.c_str(), "flames")) {
    auto handle = nanovdb::io::readGrid(filename.c_str(), "flames");
    auto grid = handle.grid<float>();
    if (grid) {
      DEBUG("[VDBMedium]", filename, " has flames attribute");
      auto [aabb, mat] = get_grid_info(grid);
      world2index_flame = mat * inverse(transform);
      flame_grid = grid;
      flame_handle = psl::make_opaque_shared_ptr<Handle>(MOVE(handle));
    }
  }
  if (nanovdb::io::hasGrid(filename.c_str(), "temperature")) {
    auto handle = nanovdb::io::readGrid(filename.c_str(), "temperature");
    auto grid = handle.grid<float>();
    if (grid) {
      DEBUG("[VDBMedium]", filename, " has temperature attribute");
      auto [aabb, mat] = get_grid_info(grid);
      world2index_tem = mat * inverse(transform);
      temperature_grid = grid;
      temperature_handle = psl::make_opaque_shared_ptr<Handle>(MOVE(handle));
    }
  }
}
psl::optional<MediumSample> VDBMedium::sample(const Ray& ray, Sampler& sampler) const {
  auto tmin = ray.tmin, tmax = ray.tmax;
  if (!bbox.intersect(ray.o, ray.d, tmin, tmax))
    return psl::nullopt;
  auto pi0 = world2index * ray(0);
  auto pi1 = world2index * ray(tmax);
  auto di = (pi1 - pi0) / tmax;
  auto density = ((nanovdb::FloatGrid*)this->density_grid)->getAccessor();

  auto rng = RNG(sampler.get1d() * float(psl::numeric_limits<uint32_t>::max()));
  auto u = rng.nextd();

  tmin = tmin * (1 + 1e-7f) + 1e-7f;
  while (auto cell = g.get_cell(pi0, di, tmin)) {
    auto sigma_maj = sigma_z_ * cell->maj;
    if (sigma_maj > 0.0f) {
      auto sigma_maj_inv = 1.0f / sigma_maj;
      while (true) {
        auto dt = -psl::log(1 - rng.nextf()) * sigma_maj_inv;
        tmin += dt;
        if (tmin >= cell->tmax)
          break;
        auto cr = pi0 + tmin * di;
        auto D = density(cr.x, cr.y, cr.z);
        auto sig_a = sigma_a_ * D;
        auto sig_s = sigma_s_ * D;
        auto sig_t = sig_a + sig_s;
        auto sig_n = sigma_maj - sig_t;
        auto prob_n = sig_n * sigma_maj_inv;
        auto prob_s = sig_s * sigma_maj_inv;
        if (u < prob_n) {  // null-scattering
          u /= prob_n;
        } else if (u < prob_n + prob_s) {  // real-scattering
          return MediumSample(ray(tmin), vec3(1.0f), 1.0f, pf);
        }
        // else {  // absorption
        //   auto blackbody_color = vec3(0.0f);
        //   if (flame_grid) {
        //     auto pc = world2index_flame * vec4(ray(tmin), 1.0f);
        //     auto flame = ((nanovdb::FloatGrid*)flame_grid)->getAccessor()(pc.x, pc.y, pc.z);
        //     blackbody_color = vec3(blackbody_intensity * flame);
        //     if (temperature_grid) {
        //       auto pc = world2index_tem * vec4(ray(tmin), 1.0f);
        //       auto T = ((nanovdb::FloatGrid*)temperature_grid)->getAccessor()(pc.x, pc.y, pc.z);
        //       blackbody_color *= blackbody(temperature_scale * 4000 * T);
        //     }
        //   }
        //   return MediumSample(ray(tmin), vec3(1.0f), sig_a * blackbody_color);
        // }
      }
    }
    tmin = cell->tmax * (1 + 1e-7f) + 1e-7f;
  }

  return psl::nullopt;
}
MediumPoint VDBMedium::at(vec3) const {
  return {.sigma_s = sigma_s};
}
vec3 VDBMedium::transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const {
  auto tmin = 0.0f;
  if (!bbox.intersect(p, d, tmin, tmax))
    return vec3(1.0f);

  auto pi0 = world2index * p;
  auto pi1 = world2index * p + d * tmax;
  auto di = (pi1 - pi0) / tmax;
  auto density = ((nanovdb::FloatGrid*)this->density_grid)->getAccessor();

  auto rng = RNG(sampler.get1d() * float(psl::numeric_limits<uint32_t>::max()));
  auto u = rng.nextd();

  while (true) {
    tmin += -psl::log(1 - rng.nextf()) * sigma_maj_inv;
    if (tmin >= tmax)
      break;
    auto cr = pi0 + di * tmin;
    auto dd = 1.0f - sigma_z_ * density(cr.x, cr.y, cr.z) * sigma_maj_inv;
    if (u < dd)
      u /= dd;
    else
      return vec3(0.0f);
  }
  return vec3(1.0f);
}

// LambdaMedium::LambdaMedium(mat4 transform, psl::function<float(vec3)> density, PhaseFunction pf,
//                            vec3 sigma_a, vec3 sigma_s)
//     : density(density),
//       bbox(OBB(AABB(vec3(-1, -1, -1), vec3(1, 1, 1)), transform)),
//       pf(pf),
//       sigma_a(sigma_a),
//       sigma_s(sigma_s),
//       sigma_z(sigma_a + sigma_s) {
//   sigma_a_ = average(sigma_a);
//   sigma_s_ = average(sigma_s);
//   sigma_z_ = sigma_a_ + sigma_s_;
//   sigma_maj = sigma_z_;
//   sigma_maj_inv = 1.0f / sigma_maj;
// }
// psl::optional<MediumSample> LambdaMedium::sample(const Ray& ray, Sampler& sampler) const {
//   (void)ray;
//   (void)sampler;
//   return psl::nullopt;
// }
// vec3 LambdaMedium::transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const {
//   auto tmin = 0.0f;
//   if (!bbox.intersect(p, d, tmin, tmax))
//     return vec3(1.0f);

//   auto rng = RNG(sampler.get1d() * float(psl::numeric_limits<uint32_t>::max()));
//   auto u0 = rng.nextd();
//   double u[]{u0, u0, u0};
//   auto tr = vec3(1.0f);

//   auto n_channel_remaining = 3;
//   bool active_channels[]{true, true, true};
//   while (n_channel_remaining) {
//     tmin += -psl::log(1 - rng.nextf()) * sigma_maj_inv;
//     if (tmin >= tmax)
//       break;
//     auto dd = vec3(1.0f) - sigma_z * density(p + tmin * d) * sigma_maj_inv;
//     for (int channel = 0; channel < 3; channel++) {
//       if (!active_channels[channel])
//         continue;
//       if (u[channel] < dd[channel]) {
//         u[channel] /= dd[channel];
//       } else {
//         tr[channel] = 0;
//         n_channel_remaining--;
//         active_channels[channel] = false;
//       }
//     }
//   }
//   return tr;
// }

void medium_context(Context& ctx) {
  ctx.type<HgPhaseFunction>("HgPF").ctor<>().ctor<float>();
  ctx.type<TwoLobeHgPhaseFunction>("Hg2PF").ctor<float, float, float>();
  ctx.type<CloudPhaseFunction>("CloudPF").ctor<>().ctor<float>();
  ctx.type<PhaseFunction>("PhaseFunction")
      .ctor_variant<HgPhaseFunction, TwoLobeHgPhaseFunction, CloudPhaseFunction>();

  ctx.type<HomogeneousMedium>("HomoMedium").ctor<Shape, PhaseFunction, vec3, vec3>();
  ctx.type<VDBMedium>("VDBMedium")
      .ctor<psl::string, mat4, PhaseFunction, vec3, vec3>()
      .ctor<psl::string, mat4, PhaseFunction, vec3, vec3, float, float>();
  //   ctx.type<LambdaMedium>("LambdaMedium")
  //       .ctor<mat4, psl::function<float(vec3)>, PhaseFunction, vec3, vec3>();
  ctx.type<Medium>("Medium").ctor_variant<HomogeneousMedium, VDBMedium>();
}

}  // namespace pine
