#include <pine/core/sampling.h>
#include <pine/core/context.h>
#include <pine/core/sampler.h>
#include <pine/core/medium.h>

#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/IO.h>

namespace pine {

PhaseFunctionSample HgPhaseFunction::sample(vec3 wi, vec2 u) const {
  auto ps = PhaseFunctionSample();
  auto cos_theta = 0.0f;
  if (std::abs(g) < 1e-3f)
    cos_theta = 1 - 2 * u[0];
  else
    cos_theta = -1 / (2 * g) * (1 + sqr(g) - sqr((1 - sqr(g)) / (1 + g - 2 * g * u[0])));

  auto sin_theta = psl::safe_sqrt(1 - sqr(cos_theta));
  auto phi = 2 * Pi * u[1];
  auto m = coordinate_system(wi);
  ps.wo = m * spherical_to_cartesian(phi, sin_theta, cos_theta);
  ps.f = ps.pdf = henyey_greenstein(dot(wi, ps.wo), g);
  return ps;
}

psl::optional<MediumSample> HomogeneousMedium::sample(vec3 p, vec3 d, float tmax,
                                                      Sampler& sampler) const {
  auto tmin = 0.0f;
  if (!aabb.intersect(Ray(p, d), tmin, tmax))
    return psl::nullopt;
  auto ms = MediumSample{.pg = HgPhaseFunction(0.0f)};
  ms.t = tmin - psl::log(1 - sampler.get1d()) / sigma_z;
  if (ms.t > tmax)
    return psl::nullopt;
  ms.tr = psl::exp(-(ms.t - tmin) * sigma_z);
  ms.pdf = sigma_z * ms.tr;
  ms.sigma = sigma_s;
  return ms;
}
vec3 HomogeneousMedium::transmittance(vec3 p, vec3 d, float tmax, Sampler&) const {
  auto tmin = 0.0f;
  if (!aabb.intersect(Ray(p, d), tmin, tmax))
    return vec3(1.0f);
  return vec3(psl::exp(-sigma_z * (tmax - tmin)));
}

VDBMedium::VDBMedium(psl::string filename, mat4 transform, float sigma_a, float sigma_s)
    : l2w(transform), w2l(inverse(l2w)), sigma_s(sigma_s), sigma_z(sigma_a + sigma_s) {
  using Handle = nanovdb::GridHandle<nanovdb::HostBuffer>;
  auto handle = nanovdb::io::readGrid(filename.c_str());
  auto grid = handle.grid<float>();
  if (!grid)
    Fatal("[VDBMedium]File content is not supported: `", filename, '`');
  this->grid = grid;
  this->handle = psl::make_opaque_shared_ptr<Handle>(psl::move(handle));
  sigma_maj = sigma_z * grid->tree().root().getMax();
  sigma_maj_inv = 1.0f / sigma_maj;
  for (int i = 0; i < 3; i++) {
    aabb.lower[i] = grid->worldBBox().min()[i];
    aabb.upper[i] = grid->worldBBox().max()[i];
    index_start[i] = grid->indexBBox().min()[i];
    index_end[i] = grid->indexBBox().max()[i];
  }
}
float VDBMedium::density(vec3 p) const {
  auto grid = (nanovdb::FloatGrid*)this->grid;
  auto density = grid->getAccessor();
  auto rp = aabb.relative_position(p);
  auto [ix, iy, iz] = psl::lerp(rp, index_start, index_end);
  return density(nanovdb::Coord(ix, iy, iz));
}
float ln(float x) {
  auto bx = psl::bitcast<uint32_t>(x);
  auto t = int32_t(bx >> 23) - 127;
  bx = 1065353216 | (bx & 8388607);
  x = psl::bitcast<float>(bx);
  return -1.49278f + (2.11263f + (-0.729104f + 0.10969f * x) * x) * x + 0.6931471806f * t;
}

psl::optional<MediumSample> VDBMedium::sample(vec3 p, vec3 d, float tmax, Sampler& sampler) const {
  auto p_end = p + d * tmax;
  p = vec3(w2l * vec4(p, 1.0f));
  p_end = vec3(w2l * vec4(p_end, 1.0f));
  d = vec3(mat3(w2l) * d);
  auto i = max_axis(abs(d));
  tmax = (p_end - p)[i] / d[i];
  auto tmin = 0.0f;
  if (!aabb.intersect(Ray(p, d), tmin, tmax))
    return psl::nullopt;

  auto ms = MediumSample{.pg = HgPhaseFunction(0.0f)};

  auto rng = RNG(sampler.get1d() * float(psl::numeric_limits<uint32_t>::max()));
  auto u = rng.nextf();
  ms.t = tmin - ln(1 - rng.nextf()) * sigma_maj_inv;
  while (ms.t < tmax) {
    auto sigma_n = sigma_maj - sigma_z * density(p + d * ms.t);
    auto dd = sigma_n * sigma_maj_inv;
    if (u < dd) {
      ms.t += -ln(1 - rng.nextf()) * sigma_maj_inv;
      u /= dd;
    } else {
      break;
    }
  }
  if (ms.t >= tmax)
    return psl::nullopt;

  ms.sigma = sigma_s;
  ms.tr = 1.0f / sigma_z;
  ms.pdf = 1.0f;
  return ms;

  // auto optic_thickness = -psl::log(1 - sampler.get1d());
  // auto dt = 3.0f * psl::max(sampler.get1d(), 0.2f) * sigma_maj_inv;
  // while (true) {
  //   if (tmin >= tmax)
  //     return psl::nullopt;
  //   auto sigma_t = sigma_z * density(p + d * (tmin + dt / 2));
  //   optic_thickness -= sigma_t * dt;
  //   if (optic_thickness <= 0) {
  //     tmin += dt + optic_thickness / sigma_t;
  //     break;
  //   } else {
  //     tmin += dt;
  //   }
  // }
  // ms.t = tmin;
  // ms.sigma = sigma_s;
  // ms.tr = 1.0f / sigma_z;
  // ms.pdf = 1.0f;
  // return ms;
}
vec3 VDBMedium::transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler [[maybe_unused]]) const {
  auto p_end = p + d * tmax;
  p = vec3(w2l * vec4(p, 1.0f));
  p_end = vec3(w2l * vec4(p_end, 1.0f));
  d = vec3(mat3(w2l) * d);
  auto i = max_axis(abs(d));
  tmax = (p_end - p)[i] / d[i];

  auto tmin = 0.0f;
  if (aabb.intersect(Ray(p, d), tmin, tmax)) {
    auto rng = RNG(sampler.get1d() * float(psl::numeric_limits<uint32_t>::max()));
    auto u = rng.nextf();
    while (true) {
      tmin += -ln(1 - rng.nextf()) * sigma_maj_inv;
      if (tmin >= tmax)
        return vec3(1.0f);

      auto dd = (sigma_maj - sigma_z * density(p + d * tmin)) * sigma_maj_inv;
      if (u <= dd)
        u /= dd;
      else
        return vec3(0.0f);
    }
  } else {
    return vec3(1.0f);
  }

  // auto tmin = 0.0f;
  // if (!aabb.intersect(Ray(p, d), tmin, tmax))
  //   return vec3(1.0f);
  // auto optic_thickness = 0.0f;
  // while (tmin < tmax) {
  //   auto t = 4.0f / (sigma_maj);
  //   optic_thickness += t * sigma_z * density(p + d * tmin);
  //   tmin += t;
  // }
  // return vec3(psl::exp(-optic_thickness));
}

void medium_context(Context& context) {
  context.type<HomogeneousMedium>("HomoMedium").ctor<AABB, float, float>();
  context.type<VDBMedium>("VDBMedium").ctor<psl::string, mat4, float, float>();
  context.type<Medium>("Medium").ctor_variant<HomogeneousMedium, VDBMedium>();
}

}  // namespace pine
