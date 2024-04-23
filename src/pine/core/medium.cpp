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

namespace pine {

HomogeneousMedium::~HomogeneousMedium() = default;

HomogeneousMedium::HomogeneousMedium(Shape shape, vec3 sigma_a, vec3 sigma_s)
    : sigma_s(sigma_s), sigma_z(sigma_a + sigma_s) {
  auto material = psl::make_shared<Material>(DiffuseMaterial(vec3(1.0f)));
  scene = psl::make_shared<Scene>();
  scene->geometries.push_back(psl::make_shared<Geometry>(shape, material));
  aabb = shape.get_aabb();
  max_dim = max_value(aabb.diagonal());
  accel = EmbreeAccel();
  accel.build(scene.get());
}
psl::optional<MediumInteraction> HomogeneousMedium::intersect_tr(const Ray& ray,
                                                                 Sampler& sampler) const {
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
        auto pdf = sigma * psl::exp(-sigma * t_sampled);
        auto W = sigma_s * exp(-sigma_z * t_sampled) / pdf;
        auto t_world = r.tmin + t_sampled - t;
        return MediumInteraction(t_world, ray(t_world), W, HgPhaseFunction(0.0f));
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

VDBMedium::VDBMedium(psl::string filename, mat4 transform, vec3 sigma_a, vec3 sigma_s,
                     float blackbody_intensity, float temperature_scale)
    : sigma_a(sigma_a),
      sigma_s(sigma_s),
      sigma_z(sigma_a + sigma_s),
      blackbody_intensity(blackbody_intensity),
      temperature_scale(temperature_scale) {
  using Handle = nanovdb::GridHandle<nanovdb::HostBuffer>;
  {
    auto handle = nanovdb::io::readGrid(filename.c_str(), "density");
    auto grid = handle.grid<float>();
    if (!grid)
      Fatal("[VDBMedium]`", filename, " `has no density attribute");
    sigma_maj = max_value(sigma_z) * grid->tree().root().getMax();
    sigma_majs = sigma_z * grid->tree().root().getMax();
    sigma_maj_inv = 1.0f / sigma_maj;
    sigma_maj_invs = 1.0f / sigma_majs;
    auto aabb = AABB();
    auto index_start = vec3(), index_end = vec3();
    for (int i = 0; i < 3; i++) {
      aabb.lower[i] = grid->worldBBox().min()[i];
      aabb.upper[i] = grid->worldBBox().max()[i];
      index_start[i] = grid->indexBBox().min()[i];
      index_end[i] = grid->indexBBox().max()[i];
    }
    bbox = OBB(aabb, transform);
    world2index = inverse(transform * translate(aabb.lower) * scale(aabb.diagonal()) *
                          scale(1.0f / (index_end - index_start)) * translate(-index_start));
    this->density_grid = grid;
    this->density_handle = psl::make_opaque_shared_ptr<Handle>(psl::move(handle));
  }
  if (nanovdb::io::hasGrid(filename.c_str(), "flames")) {
    auto handle = nanovdb::io::readGrid(filename.c_str(), "flames");
    auto grid = handle.grid<float>();
    if (grid) {
      Debug("[VDBMedium]", filename, " has flames attribute");
      auto aabb = AABB();
      auto index_start = vec3(), index_end = vec3();
      for (int i = 0; i < 3; i++) {
        aabb.lower[i] = grid->worldBBox().min()[i];
        aabb.upper[i] = grid->worldBBox().max()[i];
        index_start[i] = grid->indexBBox().min()[i];
        index_end[i] = grid->indexBBox().max()[i];
      }
      world2index_flame =
          inverse(transform * translate(aabb.lower) * scale(aabb.diagonal()) *
                  scale(1.0f / (index_end - index_start)) * translate(-index_start));
      this->flame_grid = grid;
      this->flame_handle = psl::make_opaque_shared_ptr<Handle>(psl::move(handle));
    }
  }
  if (nanovdb::io::hasGrid(filename.c_str(), "temperature")) {
    auto handle = nanovdb::io::readGrid(filename.c_str(), "temperature");
    auto grid = handle.grid<float>();
    if (grid) {
      Debug("[VDBMedium]", filename, " has temperature attribute");
      auto aabb = AABB();
      auto index_start = vec3(), index_end = vec3();
      for (int i = 0; i < 3; i++) {
        aabb.lower[i] = grid->worldBBox().min()[i];
        aabb.upper[i] = grid->worldBBox().max()[i];
        index_start[i] = grid->indexBBox().min()[i];
        index_end[i] = grid->indexBBox().max()[i];
      }
      world2index_tem = inverse(transform * translate(aabb.lower) * scale(aabb.diagonal()) *
                                scale(1.0f / (index_end - index_start)) * translate(-index_start));
      this->temperature_grid = grid;
      this->temperature_handle = psl::make_opaque_shared_ptr<Handle>(psl::move(handle));
    }
  }
}
psl::optional<MediumInteraction> VDBMedium::intersect_tr(const Ray& ray, Sampler& sampler) const {
  auto tmin = ray.tmin, tmax = ray.tmax;
  if (!bbox.intersect(ray.o, ray.d, tmin, tmax))
    return psl::nullopt;
  auto pi0 = vec3(world2index * vec4(ray(0), 1.0f));
  auto pi1 = vec3(world2index * vec4(ray(tmax), 1.0f));
  auto di = (pi1 - pi0) / tmax;
  auto density_grid = (nanovdb::FloatGrid*)this->density_grid;
  auto density = density_grid->getAccessor();

  auto rng = RNG(sampler.get1d() * float(psl::numeric_limits<uint32_t>::max()));

  auto f = vec3d(1.0f);
  auto c = int(rng.nextf() * 3);
  auto u = rng.nextd();

  while (true) {
    auto dt = -psl::log(1 - rng.nextf()) * sigma_maj_invs[c];
    tmin += dt;
    if (tmin >= tmax)
      return psl::nullopt;
    auto cr = pi0 + tmin * di;
    auto D = density(cr.x, cr.y, cr.z);
    auto sig_a = sigma_a * D;
    auto sig_s = sigma_s * D;
    auto sig_t = sig_a + sig_s;
    auto sig_n = sigma_majs - sig_t;
    auto prob_n = sig_n[c] / sigma_majs[c];
    auto prob_s = sig_s[c] / sigma_majs[c];
    if (u < prob_n) {  // null-scattering
      u /= prob_n;
      f = normalize(f * exp(-sigma_majs * dt) * sig_n);
      if (average(f) == 0.0f)
        return psl::nullopt;
    } else if (u < prob_n + prob_s) {  // real-scattering
      f = normalize(f * exp(-sigma_majs * dt) * sig_s);
      if (average(f) == 0.0f)
        return psl::nullopt;
      return MediumInteraction(tmin, ray(tmin), f / average(f), HgPhaseFunction(0.0f));
    } else {  // absorption
      f = normalize(f * exp(-sigma_majs * dt) * sig_a);
      if (average(f) == 0.0f)
        return psl::nullopt;
      auto blackbody_color = vec3(0.0f);
      auto flame_grid = (nanovdb::FloatGrid*)this->flame_grid;
      auto temperature_grid = (nanovdb::FloatGrid*)this->temperature_grid;
      if (flame_grid) {
        auto pc = world2index_flame * vec4(ray(tmin), 1.0f);
        auto flame = flame_grid->getAccessor()(pc.x, pc.y, pc.z);
        blackbody_color = vec3(blackbody_intensity * flame);
      }
      if (temperature_grid) {
        auto pc = world2index_tem * vec4(ray(tmin), 1.0f);
        auto T = temperature_grid->getAccessor()(pc.x, pc.y, pc.z);
        blackbody_color *= blackbody(temperature_scale * 4000 * T);
      }
      return MediumInteraction(f / average(f), sig_a * blackbody_color);
    }
  }
}
vec3 VDBMedium::transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const {
  auto tmin = 0.0f;
  if (!bbox.intersect(p, d, tmin, tmax))
    return vec3(1.0f);

  auto pi0 = vec3(world2index * vec4(p, 1.0f));
  auto pi1 = vec3(world2index * vec4(p + d * tmax, 1.0f));
  auto di = (pi1 - pi0) / tmax;
  auto density_grid = (nanovdb::FloatGrid*)this->density_grid;
  auto density = density_grid->getAccessor();

  auto rng = RNG(sampler.get1d() * float(psl::numeric_limits<uint32_t>::max()));
  auto u0 = rng.nextd();
  double u[]{u0, u0, u0};
  auto tr = vec3(1.0f);

  auto n_channel_remaining = 3;
  bool active_channels[]{true, true, true};
  while (n_channel_remaining) {
    tmin += -psl::log(1 - rng.nextf()) * sigma_maj_inv;
    if (tmin >= tmax)
      break;
    auto cr = pi0 + di * tmin;
    auto dd = vec3(1.0f) - sigma_z * density(cr.x, cr.y, cr.z) * sigma_maj_inv;
    for (int channel = 0; channel < 3; channel++) {
      if (!active_channels[channel])
        continue;
      if (u[channel] < dd[channel]) {
        u[channel] /= dd[channel];
      } else {
        tr[channel] = 0;
        n_channel_remaining--;
        active_channels[channel] = false;
      }
    }
  }
  return tr;
}

LambdaMedium::LambdaMedium(mat4 transform, psl::function<float(vec3)> density, vec3 sigma_a,
                           vec3 sigma_s, float blackbody_intensity, float temperature_scale)
    : density(density),
      bbox(OBB(AABB(vec3(-1, -1, -1), vec3(1, 1, 1)), transform)),
      sigma_a(sigma_a),
      sigma_s(sigma_s),
      sigma_z(sigma_a + sigma_s),
      blackbody_intensity(blackbody_intensity),
      temperature_scale(temperature_scale) {
  sigma_majs = sigma_z;
  sigma_maj_invs = vec3(1) / sigma_majs;
  sigma_maj = max_value(sigma_majs);
  sigma_maj_inv = 1.0f / sigma_maj;
}
psl::optional<MediumInteraction> LambdaMedium::intersect_tr(const Ray& ray,
                                                            Sampler& sampler) const {
  auto tmin = ray.tmin, tmax = ray.tmax;
  if (!bbox.intersect(ray.o, ray.d, tmin, tmax))
    return psl::nullopt;
  auto rng = RNG(sampler.get1d() * float(psl::numeric_limits<uint32_t>::max()));

  auto f = vec3d(1.0f);
  auto c = int(rng.nextf() * 3);
  auto u = rng.nextd();

  while (true) {
    auto dt = -psl::log(1 - rng.nextf()) * sigma_maj_invs[c];
    tmin += dt;
    if (tmin >= tmax)
      return psl::nullopt;
    auto D = density(ray(tmin));
    auto sig_a = sigma_a * D;
    auto sig_s = sigma_s * D;
    auto sig_t = sig_a + sig_s;
    auto sig_n = sigma_majs - sig_t;
    auto prob_n = sig_n[c] / sigma_majs[c];
    auto prob_s = sig_s[c] / sigma_majs[c];
    if (u < prob_n) {  // null-scattering
      u /= prob_n;
      f = normalize(f * exp(-sigma_majs * dt) * sig_n);
      if (average(f) == 0.0f)
        return psl::nullopt;
    } else if (u < prob_n + prob_s) {  // real-scattering
      f = normalize(f * exp(-sigma_majs * dt) * sig_s);
      if (average(f) == 0.0f)
        return psl::nullopt;
      return MediumInteraction(tmin, ray(tmin), f / average(f), HgPhaseFunction(0.0f));
    } else {  // absorption
      f = normalize(f * exp(-sigma_majs * dt) * sig_a);
      if (average(f) == 0.0f)
        return psl::nullopt;
      return MediumInteraction(f / average(f), sig_a);
    }
  }
}
vec3 LambdaMedium::transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const {
  auto tmin = 0.0f;
  if (!bbox.intersect(p, d, tmin, tmax))
    return vec3(1.0f);

  auto rng = RNG(sampler.get1d() * float(psl::numeric_limits<uint32_t>::max()));
  auto u0 = rng.nextd();
  double u[]{u0, u0, u0};
  auto tr = vec3(1.0f);

  auto n_channel_remaining = 3;
  bool active_channels[]{true, true, true};
  while (n_channel_remaining) {
    tmin += -psl::log(1 - rng.nextf()) * sigma_maj_inv;
    if (tmin >= tmax)
      break;
    auto dd = vec3(1.0f) - sigma_z * density(p + tmin * d) * sigma_maj_inv;
    for (int channel = 0; channel < 3; channel++) {
      if (!active_channels[channel])
        continue;
      if (u[channel] < dd[channel]) {
        u[channel] /= dd[channel];
      } else {
        tr[channel] = 0;
        n_channel_remaining--;
        active_channels[channel] = false;
      }
    }
  }
  return tr;
}

void medium_context(Context& context) {
  context.type<HomogeneousMedium>("HomoMedium").ctor<Shape, vec3, vec3>();
  context.type<VDBMedium>("VDBMedium")
      .ctor<psl::string, mat4, vec3, vec3>()
      .ctor<psl::string, mat4, vec3, vec3, float, float>();
  context.type<LambdaMedium>("LambdaMedium")
      .ctor<mat4, psl::function<float(vec3)>, vec3, vec3>()
      .ctor<mat4, psl::function<float(vec3)>, vec3, vec3, float, float>();
  context.type<Medium>("Medium").ctor_variant<HomogeneousMedium, VDBMedium, LambdaMedium>();
}

}  // namespace pine
