#include <pine/core/geometry.h>
#include <pine/core/sampling.h>
#include <pine/core/context.h>
#include <pine/core/sampler.h>
#include <pine/core/medium.h>
#include <pine/core/parallel.h>

#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/IO.h>

namespace pine {

HomogeneousMedium::HomogeneousMedium(Shape shape, vec3 sigma_a, vec3 sigma_s)
    : sigma_s(sigma_s), sigma_z(sigma_a + sigma_s) {
  auto material = psl::make_shared<Material>(DiffuseMaterial(vec3(1.0f)));
  geometry = psl::make_shared<psl::vector<psl::shared_ptr<Geometry>>>();
  geometry->push_back(psl::make_shared<Geometry>(shape, material));
  aabb = shape.get_aabb();
  max_dim = max_value(aabb.diagonal());
  accel = EmbreeAccel();
  accel.build(geometry.get());
}
void HomogeneousMedium::intersect_tr(const Ray& ray, vec3& tmax, SpectralMediumInteraction& mit,
                                     Sampler& sampler) const {
  auto inside = false;
  auto it = SurfaceInteraction();
  {
    auto r = Ray(ray.o, -ray.d, 0.0f, float_max);
    if (accel.intersect(r, it) && dot(r.d, it.n) < 0.0f)
      inside = true;
  }

  auto r = ray;
  while (true) {
    auto hit = accel.intersect(r, it);
    if (inside) {
      for (int channel = 0; channel < 3; channel++) {
        auto t = r.tmin - psl::log(1 - sampler.get1d()) / sigma_z[channel];
        if (t < tmax[channel]) {
          tmax[channel] = t;
          mit[channel] =
              MediumInteraction(ray(t), sigma_s[channel] / sigma_z[channel], HgPhaseFunction(0.0f));
        }
      }
    }
    if (!hit)
      break;

    inside = dot(r.d, it.n) < 0;
    r.tmin = r.tmax + max_dim * 1e-5f;
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
    ray.tmin = ray.tmax + 1e-3f;
    ray.tmax = tmax;
  }
}

VDBMedium::VDBMedium(psl::string filename, mat4 transform, vec3 sigma_a, vec3 sigma_s)
    : sigma_s(sigma_s), sigma_z(sigma_a + sigma_s) {
  using Handle = nanovdb::GridHandle<nanovdb::HostBuffer>;
  auto handle = nanovdb::io::readGrid(filename.c_str());
  auto grid = handle.grid<float>();
  if (!grid)
    Fatal("[VDBMedium]Expect a grid of float density from: `", filename, '`');
  this->grid = grid;
  this->handle = psl::make_opaque_shared_ptr<Handle>(psl::move(handle));
  sigma_maj = max_value(sigma_z) * grid->tree().root().getMax();
  sigma_maj_inv = 1.0f / sigma_maj;
  auto aabb = AABB();
  for (int i = 0; i < 3; i++) {
    aabb.lower[i] = grid->worldBBox().min()[i];
    aabb.upper[i] = grid->worldBBox().max()[i];
    index_start[i] = grid->indexBBox().min()[i];
    index_end[i] = grid->indexBBox().max()[i];
  }
  bbox = OBB(aabb, transform);
  world2index = inverse(transform * translate(aabb.lower) * scale(aabb.diagonal()) *
                        scale(1.0f / (index_end - index_start)) * translate(-index_start));
}
void VDBMedium::intersect_tr(const Ray& ray [[maybe_unused]], vec3& tmax [[maybe_unused]],
                             SpectralMediumInteraction& mit [[maybe_unused]],
                             Sampler& sampler [[maybe_unused]]) const {
  auto tmin = ray.tmin, tmax_ = max_value(tmax);
  if (!bbox.intersect(ray.o, ray.d, tmin, tmax_))
    return;
  auto pi0 = vec3(world2index * vec4(ray(0), 1.0f));
  auto pi1 = vec3(world2index * vec4(ray(tmax_), 1.0f));
  auto di = (pi1 - pi0) / tmax_;
  auto grid = (nanovdb::FloatGrid*)this->grid;
  auto density = grid->getAccessor();

  auto rng = RNG(sampler.get1d() * float(psl::numeric_limits<uint32_t>::max()));
  float u[3]{rng.nextf(), rng.nextf(), rng.nextf()};

  auto n_channel_remaining = 3;
  bool active_channels[]{true, true, true};
  while (n_channel_remaining) {
    tmin += -psl::log(1 - rng.nextf()) * sigma_maj_inv;
    auto cr = pi0 + tmin * di;
    auto dd = vec3(1.0f) - sigma_z * density(cr.x, cr.y, cr.z) * sigma_maj_inv;
    for (int channel = 0; channel < 3; channel++) {
      if (!active_channels[channel])
        continue;
      else if (tmin >= psl::min(tmax_, tmax[channel])) {
        n_channel_remaining--;
        active_channels[channel] = false;
      } else if (u[channel] <= dd[channel]) {
        u[channel] /= dd[channel];
      } else {
        tmax[channel] = tmin;
        mit[channel] = MediumInteraction(ray(tmin), sigma_s[channel] / sigma_z[channel],
                                         HgPhaseFunction(0.0f));
        n_channel_remaining--;
        active_channels[channel] = false;
      }
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
  auto grid = (nanovdb::FloatGrid*)this->grid;
  auto density = grid->getAccessor();

  auto rng = RNG(sampler.get1d() * float(psl::numeric_limits<uint32_t>::max()));
  auto u = rng.nextf();
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
      if (u <= dd[channel]) {
        u /= dd[channel];
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
  context.type<VDBMedium>("VDBMedium").ctor<psl::string, mat4, vec3, vec3>();
  context.type<Medium>("Medium").ctor_variant<HomogeneousMedium, VDBMedium>();
}

}  // namespace pine
