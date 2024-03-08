#include <pine/impl/integrator/cachedpath.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

#include <psl/unordered_map.h>

namespace pine {

namespace {
struct RadianceSample {
  RadianceSample() = default;
  RadianceSample(vec3 p, vec3 w, vec3 flux) : p(p), w(w), flux(flux) {
  }
  vec3 p;
  vec3 w;
  vec3 flux;
};

struct UnitVector {
  UnitVector() = default;
  UnitVector(vec3 n) {
    c[0] = psl::min((n[0] + 1) / 2 * 256, 255.0f);
    c[1] = psl::min((n[1] + 1) / 2 * 256, 255.0f);
    c[2] = psl::min((n[2] + 1) / 2 * 256, 255.0f);
  }
  vec3 decode() const {
    auto n0 = (c[0] / 255.0f) * 2 - 1;
    auto n1 = (c[1] / 255.0f) * 2 - 1;
    auto n2 = (c[2] / 255.0f) * 2 - 1;
    return vec3(n0, n1, n2);
  }

  uint8_t c[4];
};
struct UnitVectorNoOp {
  UnitVectorNoOp(vec3 n) : n(n) {
  }
  vec3 decode() const {
    return n;
  }
  vec3 n;
};

struct unsigned_float16 {
  static uint16_t float_to_bits(float x) {
    auto u = psl::bitcast<uint32_t>(x);
    auto exp = int((u >> 23) & 0b11111111);
    exp = psl::max(exp + 0b1111 - 0b1111111, 0);
    uint16_t sig = (u >> 12) & 0b11111111111;
    return (exp << 11) | sig;
  }
  static float bits_to_float(uint16_t b) {
    auto exp = uint32_t(b >> 11);
    exp = exp + 0b1111111 - 0b1111;
    auto sig = uint32_t(b & 0b11111111111) << 12;
    return psl::bitcast<float>((exp << 23) | sig);
  }

  unsigned_float16(float x) : bits(float_to_bits(x)) {
  }
  operator float() const {
    return bits_to_float(bits);
  }
  unsigned_float16& operator+=(float rhs) {
    return *this = float(*this) + rhs;
  }
  unsigned_float16& operator-=(float rhs) {
    return *this = float(*this) - rhs;
  }
  unsigned_float16& operator*=(float rhs) {
    return *this = float(*this) * rhs;
  }
  unsigned_float16& operator/=(float rhs) {
    return *this = float(*this) / rhs;
  }

  psl::string to_string() const {
    return psl::to_string(float(*this));
  }

  uint16_t bits;
};

struct SpatialNode {
  void add_flux(vec3 l) {
    lock.lock();
    flux += l;
    nsamples += 1;
    lock.unlock();
  }
  vec3 flux_estimate() const {
    if (nsamples != 0)
      return flux / nsamples;
    else
      return vec3(0);
  }

  Vector3<float> flux;
  mutable SpinLock lock;
  uint32_t nsamples;
};

struct SpatialTree {
  SpatialTree() = default;
  SpatialTree(AABB aabb, vec3i64 resolution) : aabb(aabb), resolution(resolution) {
    this->aabb.extend_by(1e-4f);
    nodes.resize(volume(resolution));
  }
  void add_sample(RadianceSample s) {
    node_at(s.p).add_flux(s.flux);
  }
  vec3 flux_estimate(vec3 p) const {
    return node_at(p).flux_estimate();
  }

private:
  SpatialNode& node_at(vec3 p) {
    auto rp = aabb.relative_position(p);
    auto ip = vec3i(rp * resolution);
    return nodes[ip.x + ip.y * resolution.x + ip.z * resolution.x * resolution.y];
  }
  const SpatialNode& node_at(vec3 p) const {
    return const_cast<SpatialTree*>(this)->node_at(p);
  }

  AABB aabb;
  vec3i resolution;
  psl::vector<SpatialNode> nodes;
};

}  // namespace

static SpatialTree stree;
static bool use_estimate = false;

CachedPathIntegrator::CachedPathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                                           int max_path_length, int max_axis_resolution,
                                           int starting_depth)
    : RTIntegrator{psl::move(accel), psl::move(sampler)},
      light_sampler{psl::move(light_sampler)},
      max_path_length{max_path_length},
      max_axis_resolution(max_axis_resolution),
      starting_depth{starting_depth} {
  if (max_path_length <= 0)
    Fatal("`CachedPathIntegrator` expect `max_path_length` to be positive, get", max_path_length);
  if (max_axis_resolution <= 0)
    Fatal("`CachedPathIntegrator` expect `max_axis_resolution` to be positive, get",
          max_axis_resolution);
  if (starting_depth < 0)
    Fatal("`CachedPathIntegrator` expect `starting_depth` to be non-negative, get", starting_depth);
}
void CachedPathIntegrator::render(Scene& scene) {
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("CachedPathIntegrator doesn't support `Plane`, please use `Rect` or `Disk` instead");

  auto aabb = scene.get_aabb();
  auto resolution = vec3i(max_axis_resolution * aabb.diagonal() / max_value(aabb.diagonal()));
  resolution = max(resolution, vec3i(1, 1, 1));
  footprint = max_value(aabb.diagonal()) / max_axis_resolution;
  stree = SpatialTree(aabb, resolution);
  accel.build(&scene);
  light_sampler.build(&scene);
  auto& film = scene.camera.film(); film.clear();
  film.clear();

  Profiler _("[CachedPath]Render");

  set_progress(0.0f);
  auto primary_spp = psl::max(spp / primary_ratio, 1);

  use_estimate = false;
  for (int i = 0; i < primary_spp; i++) {
    parallel_for(film.size(), [&](vec2i p) {
      Sampler& sampler = samplers[threadIdx];
      sampler.start_pixel(p, i * primary_ratio);
      auto p_film = vec2(p + sampler.get2d()) / scene.camera.film().size();
      auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
      radiance(scene, ray, sampler, 0, Vertex{}, primary_ratio);
      if (p.x == 0)
        set_progress(float(i) / primary_spp / 2 +
                     float(p.x + p.y * film.size().x) / area(film.size()) / primary_spp / 2);
    });
  }

  use_estimate = true;
  for (int i = 0; i < primary_spp; i++) {
    parallel_for(film.size(), [&](vec2i p) {
      Sampler& sampler = samplers[threadIdx];
      sampler.start_pixel(p, i * primary_ratio);
      auto p_film = vec2(p + sampler.get2d()) / scene.camera.film().size();
      auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
      auto L = radiance(scene, ray, sampler, 0, Vertex{}, primary_ratio);
      scene.camera.film().add_sample(p, L);
      if (p.x == 0)
        set_progress(0.5f + float(i) / primary_spp / 2 +
                     float(p.x + p.y * film.size().x) / area(film.size()) / primary_spp / 2);
    });
  }

  set_progress(1.0f);
}

vec3 CachedPathIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler, int depth, Vertex v,
                                    int ssp) {
  auto wi = -ray.d;
  auto it = Interaction{};

  if (!intersect(ray, it)) {
    if (scene.env_light) {
      auto le = scene.env_light->color(ray.d);
      if (v.bsdf_is_delta)
        return le;
      auto light_pdf = scene.env_light->pdf(v.n, ray.d);
      auto mis_term = balance_heuristic(v.sample_pdf, light_pdf);
      return le * mis_term;
    }
    return vec3(0.0f);
  }

  if (it.material()->is<EmissiveMaterial>()) {
    auto le = it.material()->le({it, wi});
    if (v.bsdf_is_delta)
      return le;
    auto light_pdf = light_sampler.pdf(it.geometry, it, ray, v.n);
    auto mis_term = balance_heuristic(v.sample_pdf, light_pdf);
    return le * mis_term;
  }

  if (depth + 1 == max_path_length)
    return vec3(0.0f);

  if (use_estimate && v.non_delta_path_length >= starting_depth)
    return stree.flux_estimate(it.p);

  auto direct_light = [&](const LightSample& ls, float pdf_g) {
    if (!hit(it.spawn_ray(ls.wo, ls.distance))) {
      auto mec = MaterialEvalCtx(it, -ray.d, ls.wo);
      auto f = it.material()->f(mec);
      auto cosine = absdot(ls.wo, it.n);
      auto mis_term = balance_heuristic(ls.pdf, pdf_g);
      return ls.le * cosine * f / ls.pdf * mis_term;
    } else {
      return vec3(0.0f);
    }
  };

  auto lo = vec3(0.0f);
  for (int sp = 0; sp < ssp; sp++) {
    if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
      auto cosine = absdot(it.n, bs->wo);
      auto li = radiance(scene, it.spawn_ray(bs->wo), sampler, depth + 1,
                         Vertex{
                             .n = it.n,
                             .bsdf_is_delta = it.material()->is_delta(),
                             .sample_pdf = bs->pdf,
                             .non_delta_path_length =
                                 v.non_delta_path_length + (it.material()->is_delta() ? 0 : 1),
                         },
                         1);
      auto sl = li * cosine * bs->f / bs->pdf;
      lo += sl;
      if (!use_estimate)
        stree.add_sample(RadianceSample(it.p, bs->wo, sl * 2));
    }

    if (!it.material()->is_delta())
      if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
        auto sl = direct_light(*ls, it.material()->pdf({it, wi, ls->wo}));
        lo += sl;
        if (!use_estimate)
          stree.add_sample(RadianceSample(it.p, ls->wo, sl * 2));
      }

    if (depth == 0)
      sampler.start_next_sample();
  }
  lo /= ssp;

  return lo;
}

}  // namespace pine