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

struct QuadTreeSample {
  vec2 sc;
  vec3 flux;
};

struct SpatialNode;
struct SpatialNode {
  SpatialNode() {
    CHECK_EQ((float)flux[0], 0);
    CHECK_EQ((float)flux[1], 0);
    CHECK_EQ((float)flux[2], 0);
    CHECK_EQ((int64_t)nsamples, 0);
  }
  void add_flux(vec3 l) {
    flux[0] += l[0];
    flux[1] += l[1];
    flux[2] += l[2];
    nsamples += 1;
  }
  vec3 flux_estimate() const {
    if (nsamples == 0)
      return vec3(0.0f);
    return vec3(flux[0], flux[1], flux[2]) / nsamples;
  }

  Atomic<float> flux[3]{};
  Atomic<int64_t> nsamples{};
};

struct SpatialTree {
  SpatialTree() = default;
  SpatialTree(AABB aabb, vec3i64 resolution) : aabb(aabb), resolution(resolution) {
    nodes.resize(volume(resolution));
  }
  void add_sample(RadianceSample s) {
    node_at(s.p).add_flux(s.flux);
  }

  vec3 flux_estimate(vec3 p) const {
    return node_at(p).flux_estimate();
  }
  int64_t sample_count_at(vec3 p) const {
    return node_at(p).nsamples;
  }

  SpatialNode& node_at(vec3 p) {
    auto rp = aabb.relative_position(p);
    auto ip = Vector3<int64_t>(rp * resolution);
    ip = clamp(ip, Vector3<int64_t>(0), resolution - Vector3<int64_t>(1));
    return nodes[ip.x + ip.y * resolution.x + ip.z * resolution.x * resolution.y];
  }
  const SpatialNode& node_at(vec3 p) const {
    return const_cast<SpatialTree*>(this)->node_at(p);
  }

private:
  AABB aabb;
  vec3i resolution;
  psl::vector<SpatialNode> nodes;
};

struct SpatialTreeHash {
  SpatialTreeHash() = default;
  SpatialTreeHash(AABB aabb, vec3i64 resolution) : aabb(aabb), resolution(resolution) {
    nodes.reserve(volume(resolution) / min_value(resolution));
  }
  void add_sample(RadianceSample s) {
    node_at(s.p).add_flux(s.flux);
  }

  vec3 flux_estimate(vec3 p) const {
    if (auto it = find_node(p))
      return it->flux_estimate();
    else
      return vec3(0.0f);
  }

  SpatialNode& node_at(vec3 p) {
    auto rp = aabb.relative_position(p);
    auto ip = vec3i64(rp * resolution);
    ip = clamp(ip, vec3i64(0), resolution - vec3i64(1));
    auto index = ip.x + ip.y * resolution.x + ip.z * resolution.x * resolution.y;
    if (auto it = nodes.find(index); it != nodes.end())
      return it->second;
    else {
      lock.lock();
      auto it_insert = nodes.insert(nodes.end(), {index, SpatialNode()});
      lock.unlock();
      return it_insert->second;
    }
  }
  const SpatialNode* find_node(vec3 p) const {
    auto rp = aabb.relative_position(p);
    auto ip = vec3i64(rp * resolution);
    DCHECK_RANGE(ip[0], 0, resolution[0] - 1);
    DCHECK_RANGE(ip[1], 0, resolution[1] - 1);
    DCHECK_RANGE(ip[2], 0, resolution[2] - 1);
    auto index = ip.x + ip.y * resolution.x + ip.z * resolution.x * resolution.y;
    if (auto it = nodes.find(index); it != nodes.end())
      return &it->second;
    else
      return nullptr;
  }

private:
  AABB aabb;
  vec3i64 resolution;
  psl::unordered_map<size_t, SpatialNode> nodes;
  SpinLock lock;
};

}  // namespace

static SpatialTreeHash sd_tree;

static bool use_estimate = false;

void CachedPathIntegrator::render(Scene& scene) {
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("CachedPathIntegrator doesn't support `Plane`, please use `Rect` or `Disk` instead");

  auto aabb = scene.get_aabb();
  auto resolution = vec3i(max_axis_resolution * aabb.diagonal() / max_value(aabb.diagonal()));
  footprint = max_value(aabb.diagonal()) / max_axis_resolution;
  sd_tree = SpatialTreeHash(aabb, resolution);
  accel.build(&scene);
  light_sampler.build(&scene);
  auto& film = scene.camera.film();
  film.clear();

  Profiler _("Rendering");

  set_progress(0.0f);

  use_estimate = false;
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = samplers[threadIdx];
    sampler.start_pixel(p, 0);
    auto p_film = vec2(p + sampler.get2d()) / scene.camera.film().size();
    auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
    auto L = radiance(scene, ray, sampler, 0, Vertex{}, int(psl::sqrt<float>(samples_per_pixel)));
    CHECK(!L.has_nan());
    CHECK(!L.has_inf());
    if (p.x == 0)
      set_progress(float(p.x + p.y * film.size().x) / area(film.size()) / 2);
  });

  use_estimate = true;
  auto primary_spp = psl::max(samples_per_pixel / primary_ratio, 1);
  for (int i = 0; i < primary_spp; i++) {
    parallel_for(film.size(), [&](vec2i p) {
      Sampler& sampler = samplers[threadIdx];
      sampler.start_pixel(p, i * primary_ratio);
      auto p_film = vec2(p + sampler.get2d()) / scene.camera.film().size();
      auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
      auto L = radiance(scene, ray, sampler, 0, Vertex{}, primary_ratio);
      CHECK(!L.has_nan());
      CHECK(!L.has_inf());
      scene.camera.film().add_sample(p, L);
      set_progress(0.5f + float(i) / primary_spp / 2 +
                   float(p.x + p.y * film.size().x) / area(film.size()) / primary_spp / 2);
    });
    set_progress(0.5f + static_cast<float>(i + 1) / primary_spp / 2);
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

  if (v.non_delta_path_length >= starting_depth && use_estimate) {
    if (filter) {
      auto tnb = coordinate_system(it.n);
      auto p = it.p + (sampler.get1d() - 0.5f) * footprint * tnb.x +
               (sampler.get1d() - 0.5f) * footprint * tnb.y;
      return min(sd_tree.flux_estimate(p), vec3(10));
    } else {
      return min(sd_tree.flux_estimate(it.p), vec3(10));
    }
  }

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
        sd_tree.add_sample(RadianceSample(it.p, bs->wo, sl * 2));
    }

    if (!it.material()->is_delta())
      if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
        auto sl = direct_light(*ls, it.material()->pdf({it, wi, ls->wo}));
        lo += sl;
        if (!use_estimate)
          sd_tree.add_sample(RadianceSample(it.p, ls->wo, sl * 2));
      }

    if (depth == 0)
      sampler.start_next_sample();
  }
  lo /= ssp;

  return lo;
}

}  // namespace pine