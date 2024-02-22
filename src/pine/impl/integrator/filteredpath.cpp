#include <pine/impl/integrator/filteredpath.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

namespace pine {

namespace {

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
  void add_sample(vec3 p, UnitVector n, UnitVector w, Vector3<unsigned_float16> l) {
    lock.lock();
    samples.emplace_back(p, n, w, l);
    lock.unlock();
  }

  psl::vector<IrradianceSample> samples;
  SpinLock lock;
};

struct SpatialTree {
  SpatialTree() = default;
  SpatialTree(AABB aabb, vec3i resolution) : aabb(aabb), resolution(resolution), nodes(resolution) {
    cube_size = aabb.diagonal() / resolution;
  }
  void add_sample(vec3 p, UnitVector n, UnitVector w, Vector3<unsigned_float16> l) {
    node_at(p).add_sample(p, n, w, l);
  }
  SpatialNode& node_at(vec3 p) {
    auto rp = aabb.relative_position(p);
    auto ip = vec3i(rp * resolution);
    ip = min(ip, resolution - vec3i(1));
    return nodes[ip];
  }
  void for_each_sample_near(vec3 p, float radius, auto f) const {
    auto r2 = radius * radius;
    auto p0 = resolution * aabb.relative_position(p - vec3(radius));
    auto p1 = resolution * aabb.relative_position(p + vec3(radius));
    auto ip0 = max(vec3i(floor(p0)), vec3i(0));
    auto ip1 = min(vec3i(ceil(p1)), resolution);
    // auto rr2 = psl::sqr(radius + max_value(cube_size) / 2 * 1.73f);
    for_3d(
        ip0, ip1, [&](vec3i ip) __attribute__((noinline)) {
          // auto x = aabb.lower + (ip + vec3(0.5f)) * cube_size;
          // if (distance_squared(p, x) > rr2)
          //   return;
          for (const auto& s : nodes[ip].samples) {
            if (distance_squared(p, s.p) < r2)
              f(s);
          }
        });
  };

private:
  AABB aabb;
  vec3i resolution;
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
  set_progress(0);
  auto filter_overhead = 40;
  auto total_samples = double(samples_per_pixel + filter_overhead) * area(film.size());

  Profiler _("[Integrator]Rendering");
  parallel_for(film.size(), [&](vec2i p) {
    auto p_film = vec2(p + vec2(0.5f)) / film.size();
    auto ray = scene.camera.gen_ray(p_film, vec2(0.5f));
    Sampler& sampler = samplers[threadIdx].start_pixel(p, 0);
    auto it = Interaction();
    auto is_hit = intersect(ray, it);
    for (int si = 0; si < samples_per_pixel; si++) {
      radiance(scene, ray, it, is_hit, sampler, Vertex::first_vertex());
      sampler.start_next_sample();
      if (p.x == 0)
        set_progress(samples_per_pixel * p.y * film.size().x / total_samples);
    }
  });

  auto current_samples = size_t(samples_per_pixel) * area(film.size());
  auto r0 = 20.0f * 2 / film.size()[0] * scene.camera.as<ThinLenCamera>().fov2d[0];
  r0 /= psl::pow(float(samples_per_pixel), 0.25f);

  parallel_for(
      film.size(), [&](vec2i p) __attribute__((noinline)) {
        Sampler& sampler = samplers[threadIdx].start_pixel(p, samples_per_pixel);
        auto p_film = vec2(p + vec2(0.5f)) / film.size();
        auto ray = scene.camera.gen_ray(p_film, vec2(0.5f));
        auto it = Interaction();
        auto Lo = vec3(0);
        auto beta = vec3(1);
        auto t = 0.0f;

        for (int i = 0; i < max_path_length; i++) {
          if (intersect(ray, it)) {
            t += ray.tmax;
            if (!it.material()->is_delta()) {
              auto total_weight = 0.0f;
              auto mec = MaterialEvalCtx(it, -ray.d, vec3(0, 0, 1));
              stree.for_each_sample_near(
                  it.p, r0 * t, [&](IrradianceSample s) __attribute__((noinline)) {
                    auto weight = 1.0f;
                    if (dot(s.n.decode(), it.n) < 0.95f)
                      weight = 0.0f;
                    if (weight == 0.0f)
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
              break;
            } else {
              if (auto bs = it.material()->sample({it, -ray.d, sampler.get1d(), sampler.get2d()})) {
                beta *= absdot(bs->wo, it.n) * bs->f / bs->pdf;
                ray = it.spawn_ray(bs->wo);
              }
            }
          } else {
            break;
          }
        }
        scene.camera.film().add_sample(p, Lo * beta);
        if (p.x == 0)
          set_progress((current_samples + filter_overhead * p.y * film.size().x) / total_samples);
      });

  set_progress(1);
}

FilteredPathIntegrator::RadianceResult FilteredPathIntegrator::radiance(Scene& scene, Ray ray,
                                                                        Interaction it, bool is_hit,
                                                                        Sampler& sampler,
                                                                        Vertex pv) const {
  auto Lo = vec3(0.0f);
  auto wi = -ray.d;

  if (pv.length != 0)
    is_hit = intersect(ray, it);

  if (!is_hit) {
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

  if (!it.material()->is_delta())
    if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
      if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
        auto cosine = absdot(ls->wo, it.n);
        auto [f, bsdf_pdf] = it.material()->f_pdf({it, wi, ls->wo});
        auto mis = ls->light->is_delta() ? 1.0f : balance_heuristic(ls->pdf, bsdf_pdf);
        Lo += ls->le * cosine * f / ls->pdf * mis;
        if (pv.non_delta_length == 0)
          stree.add_sample(it.p, it.n, ls->wo,
                           2 * ls->le / ls->pdf * (ls->light->is_delta() ? 1.0f : mis * 2));
      }
    }

  if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
    auto nv = Vertex(pv.length + 1, pv.length + (it.material()->is_delta() ? 0 : 1), it.n, it.p,
                     bs->pdf, it.material()->is_delta());
    auto [Li, mis_direct] = radiance(scene, it.spawn_ray(bs->wo), it, {}, sampler, nv);
    auto cosine = absdot(bs->wo, it.n);
    if (mis_direct) {
      Lo += Li * cosine * bs->f / bs->pdf * (*mis_direct);
      if (pv.non_delta_length == 0)
        stree.add_sample(it.p, it.n, bs->wo, 2 * Li / bs->pdf * *mis_direct * 2);
    } else {
      Lo += Li * cosine * bs->f / bs->pdf;
      if (pv.non_delta_length == 0)
        stree.add_sample(it.p, it.n, bs->wo, 2 * Li / bs->pdf);
    }
  }

  return {Lo, psl::nullopt};
}

}  // namespace pine