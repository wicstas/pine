#include <pine/core/sampling.h>
#include <pine/core/context.h>
#include <pine/core/sampler.h>
#include <pine/core/medium.h>

#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/IO.h>

namespace pine {

static float ln(float x) {
  auto bx = psl::bitcast<uint32_t>(x);
  auto t = int32_t(bx >> 23) - 127;
  bx = 1065353216 | (bx & 8388607);
  x = psl::bitcast<float>(bx);
  return -1.49278f + (2.11263f + (-0.729104f + 0.10969f * x) * x) * x + 0.6931471806f * t;
}

psl::optional<MediumInteraction> HomogeneousMedium::intersect_tr(Ray& ray, Sampler& sampler) const {
  auto tmin = ray.tmin, tmax = ray.tmax;
  if (!aabb.intersect(ray, tmin, tmax))
    return psl::nullopt;

  auto t = tmin - psl::log(1 - sampler.get1d()) / sigma_z;
  if (t > psl::min(ray.tmax, tmax))
    return psl::nullopt;

  ray.tmax = t;
  return MediumInteraction(ray(), sigma_s / sigma_z, HgPhaseFunction(0.0f));
}
vec3 HomogeneousMedium::transmittance(vec3 p, vec3 d, float tmax, Sampler&) const {
  auto tmin = 0.0f;
  if (!aabb.intersect(Ray(p, d), tmin, tmax))
    return vec3(1.0f);
  return vec3(psl::exp(-sigma_z * (tmax - tmin)));
}

VDBMedium::VDBMedium(psl::string filename, mat4 transform, float sigma_a, float sigma_s)
    : sigma_s(sigma_s), sigma_z(sigma_a + sigma_s) {
  using Handle = nanovdb::GridHandle<nanovdb::HostBuffer>;
  auto handle = nanovdb::io::readGrid(filename.c_str());
  auto grid = handle.grid<float>();
  if (!grid)
    Fatal("[VDBMedium]Expect a grid of float density from: `", filename, '`');
  this->grid = grid;
  this->handle = psl::make_opaque_shared_ptr<Handle>(psl::move(handle));
  sigma_maj = sigma_z * grid->tree().root().getMax();
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
psl::optional<MediumInteraction> VDBMedium::intersect_tr(Ray& ray, Sampler& sampler) const {
  auto tmin = ray.tmin, tmax = ray.tmax;
  if (!bbox.intersect(ray.o, ray.d, tmin, tmax))
    return psl::nullopt;
  auto pi0 = vec3(world2index * vec4(ray(0), 1.0f));
  auto pi1 = vec3(world2index * vec4(ray(tmax), 1.0f));
  auto di = (pi1 - pi0) / tmax;
  auto grid = (nanovdb::FloatGrid*)this->grid;
  auto density = grid->getAccessor();

  auto rng = RNG(sampler.get1d() * float(psl::numeric_limits<uint32_t>::max()));
  auto u = rng.nextf();
  auto t = ray.tmin - ln(1 - rng.nextf()) * sigma_maj_inv;
  while (t < tmax) {
    auto cr = pi0 + t * di;
    auto sigma_n = sigma_maj - sigma_z * density(cr.x, cr.y, cr.z);
    auto dd = sigma_n * sigma_maj_inv;
    if (u < dd) {
      t += -ln(1 - rng.nextf()) * sigma_maj_inv;
      u /= dd;
    } else {
      break;
    }
  }
  if (t >= tmax)
    return psl::nullopt;

  ray.tmax = t;
  return MediumInteraction(ray(), sigma_s / sigma_z, HgPhaseFunction(0.0f));
}
vec3 VDBMedium::transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler [[maybe_unused]]) const {
  auto tmin = 0.0f;
  if (bbox.intersect(p, d, tmin, tmax)) {
    auto pi0 = vec3(world2index * vec4(p, 1.0f));
    auto pi1 = vec3(world2index * vec4(p + d * tmax, 1.0f));
    auto di = (pi1 - pi0) / tmax;
    auto grid = (nanovdb::FloatGrid*)this->grid;
    auto density = grid->getAccessor();
    auto rng = RNG(sampler.get1d() * float(psl::numeric_limits<uint32_t>::max()));
    auto u = rng.nextf();
    while (true) {
      tmin += -ln(1 - rng.nextf()) * sigma_maj_inv;
      if (tmin >= tmax)
        return vec3(1.0f);
      auto cr = pi0 + di * tmin;
      auto dd = (sigma_maj - sigma_z * density(cr.x, cr.y, cr.z)) * sigma_maj_inv;
      if (u <= dd)
        u /= dd;
      else
        return vec3(0.0f);
    }
  } else {
    return vec3(1.0f);
  }
}

void medium_context(Context& context) {
  context.type<HomogeneousMedium>("HomoMedium").ctor<AABB, float, float>();
  context.type<VDBMedium>("VDBMedium").ctor<psl::string, mat4, float, float>();
  context.type<Medium>("Medium").ctor_variant<HomogeneousMedium, VDBMedium>();
}

}  // namespace pine
