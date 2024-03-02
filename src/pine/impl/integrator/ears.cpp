#include <pine/impl/integrator/ears.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/denoise.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

#include <psl/memory.h>
#include <psl/array.h>

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
  struct DirectionalBin {
    Vector3<Atomic<double>> estimate, moment2;
    Atomic<size_t> cost{0}, n{0};
    vec3 var_to_cost, moment2_to_cost;
  };

private:
  friend struct SpatialTree;

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
  int depth(vec3 p) {
    if (is_leaf())
      return 1;
    else
      return 1 + child(p).depth(p);
  }

  void refine(int k) {
    if (is_leaf()) {
      const auto threshold = size_t(4000 * psl::sqrt<float>(k));
      if (n_samples > threshold) {
        children = children.default_value();
        for (int i = 0; i < 8; i++) {
          child(i).n_samples = n_samples / 8;
          child(i).bins = bins;
          child(i).refine(k);
        }
      } else {
        children.reset();
      }
    } else {
      for (int i = 0; i < 8; i++)
        child(i).refine(k);
    }
    n_samples = 0;
  }
  void initial_refinement(int64_t n_samples_) {
    CHECK(is_leaf());
    n_samples = n_samples_;
    refine(1);
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
      if (p[i] < 0.5f) {
        p[i] = p[i] * 2;
      } else {
        p[i] = (p[i] - 0.5f) * 2;
        index += 1 << i;
      }
    }
    return child(index);
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
      for (int i = 0; i < 8; i++)
        child(i).for_each_non_empty_bin(f);
    }
  }

  Atomic<size_t> n_samples{0};
  psl::Array<DirectionalBin, 16> bins;
  psl::Box<psl::Array<SpatialNode, 8>> children;
};
struct SpatialTree {
  SpatialTree() = default;
  SpatialTree(AABB aabb, int64_t initial_samples) : aabb(aabb) {
    root.initial_refinement(initial_samples);
  }
  auto& traverse(vec3 p, vec3 w) {
    return root.traverse_spatial(aabb.relative_position(p)).traverse_directional(w);
  }
  int depth(vec3 p) {
    return root.depth(aabb.relative_position(p));
  }
  void refine(int k) {
    return root.refine(k);
  }
  void for_each_non_empty_bin(auto f) {
    root.for_each_non_empty_bin(f);
  }

private:
  AABB aabb;
  SpatialNode root;
};

}  // namespace

static SpatialTree stree;
static auto cost_to_var = float(0.0f);
static auto g_var = Vector3<Atomic<double>>();
static auto g_cost = Atomic<double>(0.0f);

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

void visualize(Film& film) {
  auto max_value = 0.0f;
  auto min_value = float_max;
  for_2d(film.size(), [&](vec2i p) {
    auto x = film.pixels[p].x;
    if (x == 0.0f)
      return;
    max_value = psl::max(max_value, x);
    min_value = psl::min(min_value, x);
  });
  for_2d(film.size(), [&](vec2i p) {
    auto x = film.pixels[p].x;
    if (x == 0.0f)
      return;
    film.pixels[p] = vec4(color_map((x - min_value) / (max_value - min_value)));
  });
}

void EARSIntegrator::render(Scene& scene) {
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("EARSIntegrator doesn't support `Plane`, please use `Rect` or `Disk` instead");

  accel.build(&scene);
  light_sampler.build(&scene);
  auto& film = scene.camera.film();
  film.clear();
  stree = SpatialTree(scene.get_aabb(), area(film.size()));

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

  auto si = 0;
  const auto iter_batch = 8;
  const auto is_end_of_batch = [](int iter) { return iter % iter_batch == iter_batch - 1; };
  auto iter_spp = 1;
  auto stop = false;
  auto prime_index = 0;

  for (int iter = 0; !stop; iter++) {
    if (is_end_of_batch(iter))
      iter_spp *= 2;
    auto next_iter_spp = is_end_of_batch(iter + 1) ? iter_spp * 2 : iter_spp;
    auto remaining_spp = spp - si;
    if (iter_spp + next_iter_spp > remaining_spp) {
      iter_spp = remaining_spp;
      stop = true;
    }

    estimate_ready = iter > 3;

    stats.reset();
    for (int iter_si = 0; iter_si < iter_spp; iter_si++) {
      parallel_for(film.size(), [&](vec2i p) {
        Sampler& sampler = samplers[threadIdx].start_pixel(p, si + iter_si);
        auto ray = scene.camera.gen_ray(vec2(p + sampler.get2d()) / film.size(), sampler.get2d());
        auto Ie = max(I_estimate[p], vec3(epsilon));
        auto [L, _, cost] = radiance(scene, ray, sampler, Vertex::first_vertex(Ie));
        if (iter > 0) {
          g_cost += cost;
          g_var += min(psl::sqr((L - Ie) / Ie), vec3(1e+8f));
        }
        I[p] += L;
      });
    }

    I /= iter_spp;
    if (iter == 0) {
      CHECK_EQ(g_cost, 0);
      CHECK_EQ(g_var, vec3(0));
      acc_I = I;
      denoise(DenoiseQuality::Medium, I_estimate, acc_I, albedo, normal);
      parallel_for(film.size(), [&](vec2i p) {
        auto L = I[p];
        auto Ie = max(I_estimate[p], vec3(epsilon));
        g_var += min(psl::sqr((L - Ie) / Ie), vec3(1e+8f));
      });
    }
    g_cost = g_cost / area(film.size()) / iter_spp;
    g_var = g_var / area(film.size()) / iter_spp;
    cost_to_var = g_cost / psl::max<float>(average(g_var), epsilon);
    // if (is_end_of_batch(iter))
    Debug(float(g_cost), ' ', average(g_var));

    auto weight = iter_spp / psl::max<float>(average(g_var), epsilon);
    acc_I = combine(acc_I, I, acc_weight, weight);
    acc_weight += weight;
    I.set_to_zero();
    si += iter_spp;

    if (si < 2) {
      denoise(DenoiseQuality::Medium, I_estimate, acc_I, albedo, normal);
    }
    if (prime_index >= PrimeTablesize || si >= Primes[prime_index]) {
      prime_index++;
      denoise(DenoiseQuality::Medium, I_estimate, acc_I, albedo, normal);
    }
    g_var = vec3(0);
    g_cost = 0;

    // Normalize bins statistics
    if (estimate_ready) {
      stree.for_each_non_empty_bin([&](auto& b) {
        auto n = int(b.n);
        auto cost = int(b.cost) / n;
        auto estimate = vec3(b.estimate) / n;
        auto moment2 = vec3(b.moment2) / n;
        auto var = moment2 - estimate * estimate;
        b.var_to_cost = var / cost;
        b.moment2_to_cost = moment2 / cost;
      });
    }
    stree.refine(iter_spp);
  }

  // parallel_for(film.size(), [&](vec2i p) {
  //   Sampler& sampler = samplers[threadIdx].start_pixel(p, 0);
  //   auto p_film = vec2(p + sampler.get2d()) / film.size();
  //   auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
  //   auto it = Interaction();
  //   if (intersect(ray, it)) {
  //     acc_I[p] = vec3(stree.depth(it.p));
  // auto& b = stree.traverse(it.p, -ray.d);
  // auto n = 0.0f;
  // auto throughput = vec3(1.0f) / I_estimate[p];
  // auto split_s = psl::sqrt(average(psl::sqr(throughput) * b.var_to_cost) * cost_to_var);
  // auto split_r = psl::sqrt(average(psl::sqr(throughput) * b.moment2_to_cost) * cost_to_var);
  // if (split_r > 1) {
  //   if (split_s > 1)
  //     n = split_s;
  // } else {
  //   n = split_r;
  // }
  // acc_I[p] = b.estimate / b.n;
  //   }
  // });
  stats.report();

  for_2d(film.size(), [&](vec2i p) { film[p] = vec4(acc_I[p], 1.0f); });
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
      if (!pv.is_delta)
        result.light_pdf = scene.env_light->pdf(pv.n, ray.d);
    }
    stats.record_path_length(pv.length + 1);
    LINE_PROFILE_END
    return result;
  }

  if (it.material()->is<EmissiveMaterial>()) {
    LINE_PROFILE_BEGIN  // 1
        Lo += it.material()->le({it, wi});
    if (!pv.is_delta)
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
      if (estimate_ready) {
    auto split_s = psl::sqrt(average(psl::sqr(pv.throughput) * b.var_to_cost) * cost_to_var);
    auto split_r = psl::sqrt(average(psl::sqr(pv.throughput) * b.moment2_to_cost) * cost_to_var);
    if (split_r > 1) {
      if (split_s > 1)
        n = split_s;
    } else {
      n = split_r;
    }
    n = psl::clamp(n, 0.05f, 20.0f);
    if (pv.length == 0)
      n = psl::max(n, 1.0f);
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
      auto nv = Vertex(pv.length + 1, pv.throughput / n * bs->f / bs->pdf * cosine, it.n, it.p,
                       bs->pdf, it.material()->is_delta());
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