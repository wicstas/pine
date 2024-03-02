#include <pine/impl/integrator/ears.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/denoise.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

#include <psl/memory.h>
#include <psl/array.h>

#define EARS_COLLECT_STATS 0

#if EARS_COLLECT_STATS
#define LINE_PROFILE_BEGIN [&]() __attribute__((noinline)) {
#define LINE_PROFILE_END \
  }                      \
  ();
#else
#define LINE_PROFILE_BEGIN
#define LINE_PROFILE_END
#endif

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

struct SpatialNode {
  SpatialNode() = default;
  struct DirectionalBin {
    Vector3<Atomic<float>> estimate, moment2;
    Atomic<int> cost{0}, n{0};
    vec3 var_to_cost, moment2_to_cost;
  };

private:
  friend struct SpatialTree;
  SpatialNode(AABB aabb) : axis(max_axis(aabb.diagonal())), footprint_(aabb.diagonal()[axis] / 2) {
  }

  DirectionalBin& traverse_directional(vec3 w) {
    n_samples += 1;
    auto sc = inverse_uniform_sphere(w);
    auto x = psl::min(int(sc.x * 4), 3);
    auto y = psl::min(int(sc.y * 4), 3);
    return bins[x + y * 4];
  }
  SpatialNode& traverse_spatial(vec3 p) {
    if (is_leaf()) [[unlikely]]
      return *this;
    else [[likely]]
      return child(p).traverse_spatial(p);
  }

  void refine(int k, AABB aabb) {
    if (is_leaf()) {
      const auto c = 600;
      const auto threshold = size_t(c * psl::sqrt<float>(1 << k));
      if (n_samples > threshold) {
        auto [laabb, raabb] = aabb.split_half(axis);
        auto child0 = SpatialNode(laabb);
        auto child1 = SpatialNode(raabb);
        children = psl::array_of(psl::move(child0), psl::move(child1));
        child(0).n_samples = n_samples / 2;
        child(0).bins = bins;
        child(0).refine(k, laabb);
        child(1).n_samples = n_samples / 2;
        child(1).bins = bins;
        child(1).refine(k, raabb);
      } else {
      }
    } else {
      auto [laabb, raabb] = aabb.split_half(axis);
      child(0).refine(k, laabb);
      child(1).refine(k, raabb);
    }
    n_samples = 0;
  }
  void initial_refinement(int64_t n_samples_, AABB aabb) {
    CHECK(is_leaf());
    n_samples = n_samples_;
    refine(0, aabb);
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
    if (p[axis] < 0.5f) {
      p[axis] = p[axis] * 2;
      return child(0);
    } else {
      p[axis] = (p[axis] - 0.5f) * 2;
      return child(1);
    }
  }
  const SpatialNode& child(vec3& p) const {
    return const_cast<SpatialNode*>(this)->child(p);
  }
  void for_each_non_empty_bin(auto f) {
    if (is_leaf()) {
      for (int i = 0; i < 16; i++) {
        if (bins[i].n != 0)
          f(bins[i]);
      }
    } else {
      child(0).for_each_non_empty_bin(f);
      child(1).for_each_non_empty_bin(f);
    }
  }

  int axis = -1;
  float footprint_ = 0.0f;
  Atomic<size_t> n_samples{0};
  psl::Array<DirectionalBin, 16> bins;
  psl::Box<psl::Array<SpatialNode, 2>> children;
};
struct SpatialTree {
  SpatialTree() = default;
  SpatialTree(AABB aabb) : aabb(aabb), root(aabb) {
  }
  auto& traverse(vec3 p, vec3 w) {
    return root.traverse_spatial(aabb.relative_position(p)).traverse_directional(w);
  }
  void refine(int k) {
    return root.refine(k, aabb);
  }
  void initial_refinement(int64_t n_samples_) {
    return root.initial_refinement(n_samples_, aabb);
  }
  void for_each_non_empty_bin(auto f) {
    root.for_each_non_empty_bin(f);
  }

private:
  AABB aabb;
  SpatialNode root;
};

struct SpatialGrid {
  struct DirectionalBin {
    Vector3<Atomic<double>> estimate, moment2;
    Atomic<double> cost{0}, n{0};
    vec3 var_to_cost, moment2_to_cost;
  };

  SpatialGrid() = default;
  SpatialGrid(AABB aabb, vec3i resolution) : aabb(aabb), grid(resolution) {
    this->aabb.extend_by(1e-6f);
  }
  auto& traverse(vec3 p, vec3 w);
  void refine(int) {
  }
  void initial_refinement(int64_t) {
  }
  void for_each_non_empty_bin(auto f) {
    for (auto& bin : grid)
      for (int i = 0; i < 16; i++)
        if (bin[i].n != 0)
          f(bin[i]);
  }

private:
  AABB aabb;
  Array3d<psl::Array<DirectionalBin, 16>> grid;
};

auto& SpatialGrid::traverse(vec3 p, vec3 w) {
  auto sc = inverse_uniform_sphere(w);
  auto x = psl::min(int(sc.x * 4), 3);
  auto y = psl::min(int(sc.y * 4), 3);
  return grid[aabb.relative_position(p) * grid.size()][x + y * 4];
}

}  // namespace

static SpatialGrid stree;
static auto cost_to_var = float(0.0f);
static auto g_var = Vector3<Atomic<double>>();
static auto g_cost = Atomic<double>(0.0f);

struct Statistics {
  void report() const {
    // clang-format off
    Log(
      "intersection_tests: ", intersection_tests, "\n", 
      "path_length: ", double(path_length) / path_length_n, "\n", 
      "primary_split: ", double(primary_split) / primary_split_n, "\n",
      "secondary_split: ", double(secondary_split) / secondary_split_n, "\n"
    );
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
  void record_primary_split(size_t value [[maybe_unused]]) {
#if EARS_COLLECT_STATS
    primary_split += value;
    primary_split_n += 1;
#endif
  }
  void record_secondary_split(size_t value [[maybe_unused]]) {
#if EARS_COLLECT_STATS
    secondary_split += value;
    secondary_split_n += 1;
#endif
  }

private:
  Atomic<size_t> intersection_tests{0};
  Atomic<size_t> path_length{0};
  Atomic<size_t> path_length_n{0};
  Atomic<size_t> primary_split{0};
  Atomic<size_t> primary_split_n{0};
  Atomic<size_t> secondary_split{0};
  Atomic<size_t> secondary_split_n{0};
};
static Statistics stats;

void EARSIntegrator::render(Scene& scene) {
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("EARSIntegrator doesn't support `Plane`, please use `Rect` or `Disk` instead");

  accel.build(&scene);
  light_sampler.build(&scene);
  auto& film = scene.camera.film();
  film.clear();
  stree = SpatialGrid(scene.get_aabb(), vec3i(8));
  stree.initial_refinement(1024 * 512);

  Profiler _("[Integrator]Rendering");
  set_progress(0.0f);

  // Compute aov for denoiser
  auto albedo = Array2d3f(film.size());
  auto normal = Array2d3f(film.size());
  parallel_for(film.size(), [&](vec2i p) {
    auto ray = scene.camera.gen_ray(vec2(p + vec2(0.5f)) / film.size(), vec2(0.5f));
    auto it = Interaction();
    if (intersect(ray, it)) {
      albedo[p] = it.material()->albedo({it.p, it.n, it.uv});
      normal[p] = it.n;
    }
  });

  auto acc_I = Array2d3f(film.size());
  auto acc_weight = 0.0f;
  auto I_estimate = Array2d3f(film.size());
  auto I = Array2d3f(film.size());

  // Compute the initial image estimate
  const auto initial_samples = 4;
  for (int i = 0; i < initial_samples; i++) {
    parallel_for(film.size(), [&](vec2i p) {
      Sampler& sampler = samplers[threadIdx].start_pixel(p, i);
      auto p_film = vec2(p + sampler.get2d()) / film.size();
      auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
      auto [L, mis, cost] = radiance(scene, ray, sampler, Vertex::first_vertex(0, vec3(0.0f)));
      I[p] += L;
    });
  }
  I /= initial_samples;
  denoise(DenoiseQuality::High, I_estimate, I, albedo, normal);
  Timer timer;

  stats = {};
  auto si = 0;
  auto iter_spp = 1;
  const auto iter_batch = 4;
  for (int iter = 0;; iter++) {
    if (iter % iter_batch == iter_batch - 1)
      iter_spp *= 2;
    if (si + iter_spp > spp)
      break;
    stats.reset();
    I.set_to_zero();
    cost_to_var = g_cost / average(g_var);
    g_cost = 0;
    g_var = vec3(0);
    timer.stop();
    stree.for_each_non_empty_bin([&](auto& b) {
      auto n = int(b.n);
      auto cost = int(b.cost) / n;
      auto estimate = vec3(b.estimate) / n;
      auto moment2 = vec3(b.moment2) / n;
      auto var = moment2 - estimate * estimate;
      b.var_to_cost = var / cost;
      b.moment2_to_cost = moment2 / cost;
    });
    timer.continue_();

    for (int iter_si = 0; iter_si < iter_spp; iter_si++) {
      parallel_for(film.size(), [&](vec2i p) {
        Sampler& sampler = samplers[threadIdx].start_pixel(p, initial_samples + si + iter_si);
        auto ray = scene.camera.gen_ray(vec2(p + sampler.get2d()) / film.size(), sampler.get2d());
        auto Ie = max(I_estimate[p], vec3(epsilon));
        auto [L, _, cost] = radiance(scene, ray, sampler, Vertex::first_vertex(iter, Ie));
        g_cost += cost;
        g_var += min(psl::sqr((L - Ie) / Ie), vec3(1e+8f));
        I[p] += L;
      });
    }

    g_cost = g_cost / area(film.size()) / iter_spp;
    g_var = g_var / area(film.size()) / iter_spp;
    if (iter % iter_batch == iter_batch - 1)
      Log(float(g_cost), ' ', average(g_var));

    I /= iter_spp;
    auto weight = iter_spp / average(g_var);
    acc_I = combine(acc_I, I, acc_weight, weight);
    acc_weight += weight;
    timer.stop();
    if (iter % iter_batch == iter_batch - 1)
      denoise(DenoiseQuality::Medium, I_estimate, acc_I, albedo, normal);
    timer.continue_();

    si += iter_spp;
  }

  Log("EARS: ", timer.elapsed_ms(), "ms");

// parallel_for(film.size(), [&](vec2i p) {
//   Sampler& sampler = samplers[threadIdx].start_pixel(p, 0);
//   auto p_film = vec2(p + sampler.get2d()) / film.size();
//   auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
//   auto it = Interaction();
//   if (intersect(ray, it)) {
//     auto& b = stree.traverse(it.p, -ray.d);
//     auto n = 0.0f;
//     auto throughput = vec3(1.0f) / I_estimate[p];
//     auto split_s = psl::sqrt(average(psl::sqr(throughput) * b.var_to_cost) * cost_to_var);
//     auto split_r = psl::sqrt(average(psl::sqr(throughput) * b.moment2_to_cost) * cost_to_var);
//     if (split_r > 1) {
//       if (split_s > 1)
//         n = split_s;
//     } else {
//       n = split_r;
//     }
//     acc_I[p] = b.estimate / b.n;
//   }
// });
#if EARS_COLLECT_STATS
  stats.report();
#endif

  for_2d(film.size(), [&](vec2i p) { film.add_sample_no_acc(p, acc_I[p]); });
  set_progress(1.0f);
}

EARSIntegrator::RadianceResult EARSIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler,
                                                        Vertex pv) {
  auto result = RadianceResult();
  auto wi = -ray.d;
  auto& Lo = result.Lr;
  auto& cost_sum = result.cost;

  auto it = Interaction();
  stats.record_intersection_test();
  cost_sum += 1;
  if (!intersect(ray, it)) {
    LINE_PROFILE_BEGIN  // 0
        if (scene.env_light) {
      Lo += scene.env_light->color(ray.d);
      if (pv.pdf != 0.0f)
        result.light_pdf = scene.env_light->pdf(pv.n, ray.d);
    }
    stats.record_path_length(pv.length + 1);
    LINE_PROFILE_END
    return result;
  }

  if (it.material()->is<EmissiveMaterial>()) {
    LINE_PROFILE_BEGIN  // 1
        Lo += it.material()->le({it, wi});
    if (pv.pdf != 0.0f)
      result.light_pdf = light_sampler.pdf(it.geometry, it, ray, pv.n);
    stats.record_path_length(pv.length + 1);
    LINE_PROFILE_END
    return result;
  }

  if (pv.length + 1 >= max_path_length) {
    stats.record_path_length(pv.length + 1);
    return result;
  }

  auto& b = stree.traverse(it.p, wi);
  auto n = 1.0f;
  auto ni = 1;
  LINE_PROFILE_BEGIN  // 2
      if (pv.iteration >= 3) {
    auto split_s = psl::sqrt(average(psl::sqr(pv.throughput) * b.var_to_cost) * cost_to_var);
    auto split_r = psl::sqrt(average(psl::sqr(pv.throughput) * b.moment2_to_cost) * cost_to_var);
    if (split_r > 1) {
      if (split_s > 1)
        n = split_s;
    } else {
      n = split_r;
    }
    n = psl::clamp(n, 0.05f, 20.0f);
    if (psl::fract(n) == 0.0f)
      ni = n;
    else
      ni = int(psl::floor(n)) + (sampler.get1d() < psl::fract(n) ? 1 : 0);
  }
  if (pv.length == 0) {
    stats.record_primary_split(ni);
  } else if (pv.length == 1) {
    stats.record_secondary_split(ni);
  }
  LINE_PROFILE_END

  for (int i = 0; i < ni; i++) {
    auto W = vec3(0.0f);
    auto cost = 0.0f;
    LINE_PROFILE_BEGIN  // 3
        if (!it.material()->is_delta()) {
      if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
        if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
          auto cosine = absdot(ls->wo, it.n);
          if (ls->light->is_delta()) {
            auto f = it.material()->f({it, wi, ls->wo});
            W += ls->le * cosine * f / ls->pdf;
          } else {
            auto [f, bsdf_pdf] = it.material()->f_pdf({it, wi, ls->wo});
            auto mis = balance_heuristic(ls->pdf, bsdf_pdf);
            W += ls->le * cosine * f / ls->pdf * mis;
          }
        }
      }
      cost += 1;
    }
    LINE_PROFILE_END

    LINE_PROFILE_BEGIN  // 4
        if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
      auto cosine = absdot(bs->wo, it.n);
      auto nv = Vertex(pv.iteration, pv.length + 1, pv.throughput / n * bs->f / bs->pdf * cosine,
                       it.n, it.p, bs->pdf, it.material()->is_delta());
      auto [Li, light_pdf, Li_cost] = radiance(scene, it.spawn_ray(bs->wo), sampler, nv);
      auto mis = 1.0f;
      if (light_pdf)
        mis = balance_heuristic(bs->pdf, *light_pdf);
      W += Li * cosine * bs->f / bs->pdf * mis;
      cost += Li_cost;
    }
    Lo += W;
    b.cost += cost;
    b.estimate += W;
    b.moment2 += W * W;
    b.n += 1;
    cost_sum += cost;
    LINE_PROFILE_END
  }
  Lo /= n;

  if (ni == 0) {
    stats.record_path_length(pv.length + 1);
  }

  return result;
}

}  // namespace pine