#include <pine/impl/integrator/filteredpath.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

namespace pine {

namespace {

struct UnitVector {
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
  unsigned_float16(float x) : bits(float_to_bits(x)) {
  }
  operator float() const {
    return bits_to_float(bits);
  }

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
  psl::string to_string() const {
    return psl::to_string(float(*this));
  }

  uint16_t bits;
};

struct IrradianceSample {
  vec3 p;
  UnitVector n;
  UnitVector w;
  Vector3<unsigned_float16> l;
};

struct SpatialNode {
  void add_sample(IrradianceSample s) {
    samples.push_back(s);
  }
  void add_sample(vec3 p, vec3 n, vec3 w, vec3 l) {
    samples.emplace_back(p, n, w, l);
  }

  psl::vector<IrradianceSample> samples;
};

struct SpatialTree {
  struct IndexedIrradianceSample : IrradianceSample {
    size_t index;
  };
  SpatialTree() = default;
  SpatialTree(AABB aabb, vec3i resolution) : aabb(aabb), resolution(resolution), nodes(resolution) {
    cube_size = aabb.diagonal() / resolution;
  }
  void reserve_samples(size_t n) {
    Debug("Reserving ", psl::max(n / 1000000, size_t(1)), "M samples");
    samples.reserve(n);
  }
  void add_sample(vec3 p, vec3 n, vec3 w, vec3 li) {
    samples[sample_index++] = IndexedIrradianceSample{{p, n, w, li}, index_at(p)};
  }
  void populate_cells() {
    Profiler _("[FilteredPath]Populate cells");
    auto size = psl::exchange(sample_index, 0);
    for (int64_t i = 0; i < size; i++)
      nodes.data()[samples[i].index].add_sample(samples[i]);
  };

  size_t index_at(vec3 p) {
    auto rp = aabb.relative_position(p);
    auto ip = vec3i(rp * resolution);
    ip = min(ip, resolution - vec3i(1));
    return nodes.index(ip);
  }
  void for_each_sample_near(vec3 p, float radius, auto f) const {
    auto r2 = radius * radius;
    auto p0 = resolution * aabb.relative_position(p - vec3(radius));
    auto p1 = resolution * aabb.relative_position(p + vec3(radius));
    auto ip0 = max(vec3i(floor(p0)), vec3i(0));
    auto ip1 = min(vec3i(ceil(p1)), resolution);
    auto rr2 = psl::sqr(radius + max_value(cube_size) / 2 * 1.73f);
    for_3d(ip0, ip1, [&](vec3i ip) {
      auto x = aabb.lower + (ip + vec3(0.5f)) * cube_size;
      // if (distance_squared(p, x) > rr2)
      // return;
      for (const auto& s : nodes[ip].samples) {
        if (distance_squared(p, s.p) < r2)
          f(s);
      }
    });
  };

private:
  AABB aabb;
  vec3i resolution;
  psl::vector<IndexedIrradianceSample> samples;
  Atomic<int64_t> sample_index{0};
  Array3d<SpatialNode> nodes;
  vec3 cube_size;
};

}  // namespace

static SpatialTree stree;

void FilteredPathIntegrator::render(Scene& scene) {
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("FilteredPathIntegrator doesn't support `Plane`, please use `Rect` or `Disk` instead");

  auto aabb = scene.get_aabb();
  int max_axis_resolution = 256;
  auto resolution = vec3i(max_axis_resolution * aabb.diagonal() / max_value(aabb.diagonal()));
  stree = SpatialTree(aabb, resolution);

  accel.build(&scene);
  light_sampler.build(&scene);
  auto& film = scene.camera.film();
  film.clear();
  set_progress(0);

  stree.reserve_samples(samples_per_pixel * area(film.size()) * (max_path_length - 1) * 2);

  Profiler _("[Integrator]Rendering");
  for (int si = 0; si < samples_per_pixel; si++) {
    parallel_for(film.size(), [&](vec2i p) {
      Sampler& sampler = samplers[threadIdx].start_pixel(p, si);
      auto p_film = vec2(p + sampler.get2d()) / scene.camera.film().size();
      auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
      auto [L, _] = radiance(scene, ray, sampler, Vertex::first_vertex());
    });
    set_progress(static_cast<float>(si) / samples_per_pixel);
  }

  stree.populate_cells();

  auto r0 = 10.0f * 2 / film.size()[0] * scene.camera.as<ThinLenCamera>().fov2d[0];
  r0 /= psl::pow(float(samples_per_pixel), 0.25f);

  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = samplers[threadIdx].start_pixel(p, samples_per_pixel);
    auto p_film = vec2(p) / scene.camera.film().size();
    auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
    auto it = Interaction();
    auto Lo = vec3(0);
    auto t = 0.0f;

  ray_begin:
    if (intersect(ray, it)) {
      t += ray.tmax;
      if (!it.material()->is_delta()) {
        auto total_weight = 0.0f;
        auto mec = MaterialEvalCtx(it, -ray.d, vec3(0, 0, 1));
        stree.for_each_sample_near(it.p, r0 * t, [&](IrradianceSample s) {
          auto weight = 1.0f;
          if (dot(s.n.decode(), it.n) < 0.95f)
            weight = 0.0f;
          if (weight < 0.01f)
            return;
          total_weight += weight;
          auto wo = s.w.decode();
          auto cosine = absdot(it.n, wo);
          mec.wo = it.to_local(wo);
          auto f = it.material()->f(mec);
          Lo += s.l * cosine * f * weight;
        });
        if (total_weight > 0)
          Lo /= total_weight;
      } else {
        if (auto bs = it.material()->sample({it, -ray.d, sampler.get1d(), sampler.get2d()})) {
          ray = it.spawn_ray(bs->wo);
          goto ray_begin;
        }
      }
    }
    scene.camera.film().add_sample(p, Lo);
  });

  set_progress(1);
}

FilteredPathIntegrator::RadianceResult FilteredPathIntegrator::radiance(Scene& scene, Ray ray,
                                                                        Sampler& sampler,
                                                                        Vertex pv) const {
  auto Lo = vec3(0.0f);
  auto wi = -ray.d;
  auto it = Interaction();

  if (!intersect(ray, it)) {
    if (scene.env_light) {
      Lo += scene.env_light->color(ray.d);
      if (!pv.is_delta) {
        auto light_pdf = scene.env_light->pdf(pv.n, ray.d);
        auto mis_term = balance_heuristic(pv.pdf, light_pdf);
        return {Lo, mis_term};
      }
    }
    return {Lo, psl::nullopt};
  }

  if (it.material()->is<EmissiveMaterial>()) {
    Lo += it.material()->le({it, wi});
    if (!pv.is_delta) {
      auto light_pdf = light_sampler.pdf(it.geometry, it, ray, pv.n);
      auto mis_term = balance_heuristic(pv.pdf, light_pdf);
      return {Lo, mis_term};
    }
    return {Lo, psl::nullopt};
  }

  if (pv.length + 1 >= max_path_length)
    return {Lo, psl::nullopt};

  // TODO: handle delta light source
  if (!it.material()->is_delta())
    if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
      if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
        auto cosine = absdot(ls->wo, it.n);
        auto [f, bsdf_pdf] = it.material()->f_pdf({it, wi, ls->wo});
        auto mis = ls->light->is_delta() ? 1.0f : balance_heuristic(ls->pdf, bsdf_pdf);
        Lo += ls->le * cosine * f / ls->pdf * mis;
        stree.add_sample(it.p, it.n, ls->wo,
                         2 * ls->le / ls->pdf * (ls->light->is_delta() ? 1.0f : mis * 2));
      }
    }

  if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
    auto nv = Vertex(pv.length + 1, it.n, it.p, bs->pdf, it.material()->is_delta());
    auto [Li, mis_direct] = radiance(scene, it.spawn_ray(bs->wo), sampler, nv);
    auto cosine = absdot(bs->wo, it.n);
    if (mis_direct) {
      Lo += Li * cosine * bs->f / bs->pdf * (*mis_direct);
      stree.add_sample(it.p, it.n, bs->wo, 2 * Li / bs->pdf * *mis_direct * 2);
    } else {
      Lo += Li * cosine * bs->f / bs->pdf;
      stree.add_sample(it.p, it.n, bs->wo, 2 * Li / bs->pdf);
    }
  }

  return {Lo, psl::nullopt};
}

}  // namespace pine