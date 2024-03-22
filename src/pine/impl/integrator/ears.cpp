#include <pine/impl/integrator/ears.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/denoise.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

#include <psl/memory.h>
#include <psl/array.h>

#include <algorithm>

namespace pine {

EARSIntegrator::EARSIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                               int max_path_length)
    : RTIntegrator{psl::move(accel), psl::move(sampler)},
      light_sampler{psl::move(light_sampler)},
      max_path_length{max_path_length} {
  if (max_path_length <= 0)
    Fatal("`EARSIntegrator` expect `max_path_length` to be positive, get", max_path_length);
}

namespace {

#define EARS_COLLECT_STATS 1

#if EARS_COLLECT_STATS
#define LINE_PROFILE_BEGIN [&]() __attribute__((noinline)) {
#define LINE_PROFILE_END \
  }                      \
  ();
#else
#define LINE_PROFILE_BEGIN
#define LINE_PROFILE_END
#endif

struct Statistics {
  void report() const {
    // clang-format off
#if EARS_COLLECT_STATS
    Log(
      "intersection_tests: ", intersection_tests, "\n", 
      "path_length: ", double(path_length) / path_length_n, "\n", 
      "primary_split: ", double(primary_split) / primary_split_n, "\n",
      "secondary_split: ", double(secondary_split) / secondary_split_n, "\n"
    );
#endif
    // clang-format on
  }
  void reset() {
    path_length = 0;
    path_length_n = 0;
    primary_split = 0;
    primary_split_n = 0;
    secondary_split = 0;
    secondary_split_n = 0;
  }
  void record_intersection_test() {
#if EARS_COLLECT_STATS
    intersection_tests++;
#endif
  }
  void record_path_length(size_t value [[maybe_unused]]) {
#if EARS_COLLECT_STATS
    path_length += value;
    path_length_n += 1;
#endif
  }
  void record_primary_split(double value [[maybe_unused]]) {
#if EARS_COLLECT_STATS
    primary_split += value;
    primary_split_n += 1;
#endif
  }
  void record_secondary_split(double value [[maybe_unused]]) {
#if EARS_COLLECT_STATS
    secondary_split += value;
    secondary_split_n += 1;
#endif
  }

private:
  Atomic<size_t> intersection_tests{0};
  Atomic<size_t> path_length{0};
  Atomic<size_t> path_length_n{0};
  Atomic<double> primary_split{0};
  Atomic<size_t> primary_split_n{0};
  Atomic<double> secondary_split{0};
  Atomic<size_t> secondary_split_n{0};
};
static Statistics global_stats;

struct SpatialNode {
  struct DirectionalBin {
    float splitting_factor(vec3 throughput, float g_cost_to_var) const {
      auto n = 1.0f;
      auto split_s = average(psl::sqr(throughput) * var_to_cost) * g_cost_to_var;
      auto split_r = average(psl::sqr(throughput) * moment2_to_cost) * g_cost_to_var;
      if (split_r > 1) {
        if (split_s > 1)
          n = split_s;
      } else {
        n = split_r;
      }
      return psl::sqrt(n);
    }
    Vector3<Atomic<float>> estimate, moment2;
    Atomic<float> cost{0};
    Atomic<uint32_t> n{0};
    bool valid = false;
    vec3 var_to_cost, moment2_to_cost;
  };

private:
  friend struct SpatialTree;

  DirectionalBin& traverse_directional(vec3 w) {
    n_samples += 1;
    auto sc = inverse_uniform_sphere(w);
    auto x = psl::min(int(sc.x * bin_resolution), bin_resolution - 1);
    auto y = psl::min(int(sc.y * bin_resolution), bin_resolution - 1);
    return bins[x + y * bin_resolution];
  }
  SpatialNode& traverse_spatial(vec3 p) {
    if (is_leaf()) [[unlikely]]
      return *this;
    else [[likely]]
      return child(p).traverse_spatial(p);
  }
  int depth(vec3 p) {
    if (is_leaf()) [[unlikely]]
      return 1;
    else [[likely]]
      return 1 + child(p).depth(p);
  }

  void refine(size_t threshold) {
    if (is_leaf()) {
      if (n_samples > threshold) {
        children = children.default_value();
        auto decay_factor = 1;
        for (auto& bin : bins) {
          bin.cost = bin.cost / 8 / decay_factor;
          bin.estimate = bin.estimate / 8 / decay_factor;
          bin.moment2 = bin.moment2 / 8 / decay_factor;
          bin.n = bin.n / 8 / decay_factor;
        }
        for (int i = 0; i < 8; i++) {
          child(i).footprint = footprint / 2;
          child(i).n_samples = n_samples / 8;
          child(i).bins = bins;
          child(i).refine(threshold);
        }
      }
    } else {
      for (int i = 0; i < 8; i++)
        child(i).refine(threshold);
    }
    n_samples = 0;
  }
  void initial_refinement(int64_t initial_samples, size_t threshold) {
    CHECK(is_leaf());
    n_samples = initial_samples;
    refine(threshold);
    n_samples = 0;
  }
  bool is_leaf() const {
    return !children;
  }
  SpatialNode& child(int index) {
    return (*children)[index];
  }
  const SpatialNode& child(int index) const {
    return (*children)[index];
  }
  SpatialNode& child(vec3& p) {
    auto index = 0;
#pragma unroll
    for (int i = 0; i < 3; i++) {
      auto bit = int(p[i] >= 0.5f);
      p[i] = p[i] * 2 - bit;
      index += bit << i;
    }
    return child(index);
  }
  const SpatialNode& child(vec3& p) const {
    return const_cast<SpatialNode*>(this)->child(p);
  }
  void for_each_bin(auto f) {
    if (is_leaf()) {
      for (auto& bin : bins)
        f(bin);
    } else {
      for (int i = 0; i < 8; i++)
        child(i).for_each_bin(f);
    }
  }

  float footprint = 0.0f;
  Atomic<size_t> n_samples{0};
  static constexpr int bin_resolution = 4;
  psl::Array<DirectionalBin, bin_resolution * bin_resolution> bins;
  psl::Box<psl::Array<SpatialNode, 8>> children;
};
struct SpatialTree {
  SpatialTree() = default;
  SpatialTree(AABB aabb, int64_t initial_samples) : aabb(aabb) {
    root.initial_refinement(initial_samples, node_factor);
    root.footprint = min_value(aabb.diagonal());
  }
  auto& traverse(vec3 p, vec3 w) {
    return root.traverse_spatial(aabb.relative_position(p)).traverse_directional(w);
  }
  auto& traverse(vec3 p, vec3 w, vec3 u) {
    auto footprint = root.traverse_spatial(aabb.relative_position(p)).footprint;
    p += footprint * (u - vec3(0.5f));
    p = clamp(p, aabb.lower, aabb.upper);
    return root.traverse_spatial(aabb.relative_position(p)).traverse_directional(w);
  }
  int depth(vec3 p) {
    return root.depth(aabb.relative_position(p));
  }
  void refine(int spp) {
    return root.refine(node_factor * psl::pow<float>(spp, 0.3f));
  }
  void for_each_bin(auto f) {
    root.for_each_bin(f);
  }

  static constexpr int node_factor = 20000;

private:
  AABB aabb;
  SpatialNode root;
};

}  // namespace

static SpatialTree stree;
static auto cost_to_var = float(0.0f);
static auto g_var = Vector3<Atomic<double>>();
static auto g_cost = Atomic<double>(0.0f);

struct EARSIntegrator::Vertex {
  Vertex(int length, vec3 throughput, vec3 throughput_abs, vec3 n, vec3 p, float pdf,
         bool is_delta = false)
      : length(length),
        throughput(throughput),
        throughput_abs(throughput_abs),
        n(n),
        p(p),
        pdf(pdf),
        is_delta(is_delta) {
  }
  static Vertex first_vertex(vec3 I) {
    return Vertex(0, vec3(1.0f) / I, vec3(1.0f) / I, vec3(0), vec3(0), 0.0f, true);
  }
  int length;
  vec3 throughput;
  vec3 throughput_abs;
  vec3 n;
  vec3 p;
  float pdf;
  bool is_delta;
};

void EARSIntegrator::render(Scene& scene) {
  RTIntegrator::render(scene);
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("EARSIntegrator doesn't support `Plane`, please use `Rect` or `Disk` instead");

  light_sampler.build(&scene);
  auto& film = scene.camera.film();
  Film debug_film = film;
  stree = SpatialTree(scene.get_aabb(), area(film.size()));

  set_progress(0.0f);

  // For denoising later
  auto albedo = Array2d3f(film.size());
  auto normal = Array2d3f(film.size());
  parallel_for(film.size(), [&](vec2i p) {
    auto ray = scene.camera.gen_ray((p + vec2(0.5f)) / film.size(), vec2(0.5f));
    auto it = SurfaceInteraction();
    if (intersect(ray, it)) {
      albedo[p] = it.material().albedo({it.p, it.n, it.uv});
      normal[p] = it.n;
    }
  });

  auto acc_I = Array2d3f(film.size());
  auto acc_weight = 0.0f;
  auto I_estimate = Array2d3f(film.size());
  auto I = Array2d3f(film.size());

  auto si = 0;
  const auto iter_batch = 8;
  const auto is_end_of_batch = [](int iter) { return iter % iter_batch == iter_batch - 1; };
  auto iter_spp = 1;
  auto stop = false;
  auto prime_index = 2;

  Profiler _("[EARS]Render");
  for (int iter = 0; !stop; iter++) {
    if (is_end_of_batch(iter))
      iter_spp *= 2;
    auto next_iter_spp = is_end_of_batch(iter + 1) ? iter_spp * 2 : iter_spp;
    auto remaining_spp = spp - si;
    if (iter_spp + next_iter_spp > remaining_spp) {
      iter_spp = remaining_spp;
      stop = true;
    }

    global_stats.reset();
    g_var = {};
    g_cost = {};
    auto vars = psl::vector<vec3>(area(film.size()));
    parallel_for(film.size(), [&](vec2i p) {
      Sampler& sampler = samplers[threadIdx];
      auto Ie = I_estimate[p];
      if (iter > 4 && average(Ie) < 0.001f)
        return;
      auto cost_total = 0.0f;
      auto L_total = vec3(0.0f);
      debug_film[p] = {};
      for (int iter_si = 0; iter_si < iter_spp; iter_si++) {
        sampler.start_pixel(p, si + iter_si);
        // auto ray = scene.camera.gen_ray((p + sampler.get2d()) / film.size(), sampler.get2d());
        auto ray = scene.camera.gen_ray((p + vec2(0.5f)) / film.size(), vec2(0.5f));
        auto stats = Stats();
        auto [L, _, cost] =
            radiance(scene, ray, sampler, Vertex::first_vertex(max(Ie, vec3(1e-2f))), stats);
        L_total += L;
        cost_total += cost;
        debug_film.add_sample(p, stats.value);
      }
      I[p] = L_total / iter_spp;
      g_cost += cost_total / iter_spp;
      vars[p.x + film.size().x * p.y] = psl::sqr((I[p] - Ie) / max(Ie, vec3(1e-2f))) * iter_spp;
    });
    if (iter == 0) {
      denoise(DenoiseQuality::Medium, I_estimate, I, albedo, normal);
      parallel_for(film.size(), [&](vec2i p) {
        auto Ie = I_estimate[p];
        vars[p.x + film.size().x * p.y] = psl::sqr((I[p] - Ie) / max(Ie, vec3(1e-2f))) * iter_spp;
      });
    }

    g_cost = g_cost / area(film.size());
    g_var = g_var / area(film.size());
    {
      Profiler _("Sorting");
      std::sort(vars.begin(), vars.end(), [](vec3 a, vec3 b) { return average(a) < average(b); });
    }
    auto N = area(film.size()) * (1 - 0.00001f);
    g_var = psl::mean<vec3d>(psl::trim(vars, 0, N));
    cost_to_var = g_cost / psl::max<float>(average(g_var), epsilon);
    Debug(si + iter_spp, " cost: ", float(g_cost), " variance: ", float(average(g_var)),
          " efficiency: ", 100.0f / float(g_cost * average(g_var)));

    auto weight = iter_spp / psl::max<float>(average(g_var), epsilon);
    combine_inplace(acc_I, I, acc_weight, weight);
    if (iter > 0)
      acc_weight += weight;
    si += iter_spp;

    if (si >= prime_index * prime_index) {
      prime_index++;
      denoise(DenoiseQuality::Medium, I_estimate, acc_I, albedo, normal);
    }

    stree.refine(iter_spp);
    // // Normalize bins statistics
    stree.for_each_bin([&](auto& b) {
      if (b.n > stree.node_factor / 4 / 16) {
        auto n = size_t(b.n);
        auto cost = float(b.cost) / n;
        auto estimate = vec3(b.estimate / n);
        auto moment2 = vec3(b.moment2 / n);
        auto var = moment2 - estimate * estimate;
        b.var_to_cost = var / cost;
        b.moment2_to_cost = moment2 / cost;
        // Moving average
        b.cost = b.cost * 0.5f;
        b.estimate = b.estimate * 0.5f;
        b.moment2 = b.moment2 * 0.5f;
        b.n = b.n * 0.5f;
        // b.cost = {};
        // b.estimate = {};
        // b.moment2 = {};
        // b.n = {};
        b.valid = true;
      } else {
        b.valid = false;
      }
    });
  }

  film.pixels = Array2d4f::from(acc_I);
  // film = debug_film;
  global_stats.report();
  set_progress(1.0f);
}

EARSIntegrator::RadianceResult EARSIntegrator::radiance(Scene& scene [[maybe_unused]],
                                                        Ray ray [[maybe_unused]],
                                                        Sampler& sampler [[maybe_unused]],
                                                        Vertex pv [[maybe_unused]],
                                                        Stats& stats [[maybe_unused]]) {
  return {};
  // auto result = RadianceResult();
  // auto wi = -ray.d;
  // auto& Lo = result.Lo;

  // auto it = SurfaceInteraction();
  // global_stats.record_intersection_test();
  // result.cost += 1;
  // if (pv.length == 1)
  //   result.cost += 10;
  // if (!intersect(ray, it)) {
  //   if (scene.env_light) {
  //     Lo += scene.env_light->color(ray.d);
  //     if (!pv.is_delta)
  //       result.light_pdf = scene.env_light->pdf(pv.n, ray.d);
  //   }
  //   global_stats.record_path_length(pv.length + 1);
  //   return result;
  // }

  // if (it.material().is<EmissiveMaterial>()) {
  //   Lo += it.material().le({it, wi});
  //   if (!pv.is_delta)
  //     result.light_pdf = light_sampler.pdf(it.geometry, it, ray, pv.n);
  //   global_stats.record_path_length(pv.length + 1);
  //   return result;
  // }

  // if (pv.length + 1 >= max_path_length) {
  //   global_stats.record_path_length(pv.length + 1);
  //   return result;
  // }

  // auto& b = stree.traverse(it.p, wi);
  // auto n = 1.0f;
  // auto ni = 1;
  // if (b.valid) {
  //   n = b.splitting_factor(pv.length == 1 ? pv.throughput : pv.throughput, cost_to_var);
  //   n = psl::clamp(n, 0.05f, 20.0f);
  //   if (psl::fract(n) == 0.0f)
  //     ni = n;
  //   else
  //     ni = int(psl::floor(n)) + (sampler.get1d() < psl::fract(n) ? 1 : 0);
  //   const auto stat_depth = 0;
  //   if (pv.length == stat_depth) {
  //     stats.value = color_map(b.splitting_factor(pv.throughput, cost_to_var) / 10.0f);
  //   }
  // }
  // if (pv.length == 0)
  //   global_stats.record_primary_split(n);
  // else if (pv.length == 1)
  //   global_stats.record_secondary_split(n);

  // auto b_cost = 0.0f;
  // auto estimate = vec3(0);
  // auto moment2 = vec3(0);

  // for (int i = 0; i < ni; i++) {
  //   auto W = vec3(0.0f);
  //   auto cost = 0.0f;
  //   if (!it.material().is_delta()) {
  //     if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
  //       if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
  //         auto cosine = absdot(ls->wo, it.n);
  //         if (ls->light->is_delta()) {
  //           auto f = it.material().f({it, wi, ls->wo});
  //           W += ls->le * cosine * f / ls->pdf;
  //         } else {
  //           auto [f, bsdf_pdf] = it.material().f_pdf({it, wi, ls->wo});
  //           auto mis = balance_heuristic(ls->pdf, bsdf_pdf);
  //           W += ls->le * cosine * f / ls->pdf * mis;
  //         }
  //       }
  //     }
  //     cost += 1.0f;
  //   }

  //   if (auto bs = it.material().sample({it, wi, sampler.get1d(), sampler.get2d()})) {
  //     auto cosine = absdot(bs->wo, it.n);
  //     auto nv = Vertex(pv.length + 1, pv.throughput / n * bs->f / bs->pdf * cosine,
  //                      pv.throughput * bs->f / bs->pdf * cosine, it.n, it.p, bs->pdf,
  //                      it.material().is_delta());
  //     auto [Li, light_pdf, Li_cost] = radiance(scene, it.spawn_ray(bs->wo), sampler, nv, stats);
  //     auto mis = 1.0f;
  //     if (light_pdf)
  //       mis = balance_heuristic(bs->pdf, *light_pdf);
  //     W += Li * cosine * bs->f / bs->pdf * mis;
  //     cost += Li_cost;
  //   }
  //   Lo += W;
  //   b_cost += cost;
  //   estimate += W;
  //   moment2 += W * W;
  // }
  // Lo /= n < 1 ? n : ni;
  // if (ni != 0) {
  //   b.cost += b_cost;
  //   b.estimate += estimate;
  //   b.moment2 += moment2;
  //   b.n += ni;
  //   result.cost += b_cost;
  // }

  // if (ni == 0) {
  //   global_stats.record_path_length(pv.length + 1);
  // }

  // return result;
}

}  // namespace pine