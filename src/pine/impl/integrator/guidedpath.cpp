#include <pine/impl/integrator/spatial_tree.h>
#include <pine/impl/integrator/guidedpath.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/denoise.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

#include <algorithm>

namespace pine {

static SpatialTree guide;

GuidedPathIntegrator::GuidedPathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                                           int max_path_length)
    : RTIntegrator{psl::move(accel), psl::move(sampler)},
      light_sampler{psl::move(light_sampler)},
      max_path_length{max_path_length} {
  if (max_path_length <= 0)
    Fatal("`GuidedPathIntegrator` expect `max_path_length` to be positive, get", max_path_length);
}

struct IterativeScheme {
  struct Iteration {
    vec2i image_size;
    int sample_index;
    int spp;
    float progress_start;
    float progress_end;
    int number;
    int full_iter_number;
    bool is_final;
    bool prime_hit;
  };

  IterativeScheme(vec2i image_size, size_t initial_samples, int spp, int batch_size)
      : image_size(image_size), total_samples(size_t(area(image_size)) * spp) {
    auto current_samples = size_t(0);
    auto current_sample_index = 0;
    auto iter_multiplicity = 1;
    auto prime_index = 1;

    auto full_iter = 0;
    auto is_final_iteration = false;
    for (int iter = 0; !is_final_iteration; iter++) {
      if ((iter) % batch_size == batch_size - 1)
        iter_multiplicity *= 2;
      auto next_iter_multiplicity = iter_multiplicity;
      if ((iter + 1) % batch_size == batch_size - 1)
        next_iter_multiplicity *= 2;

      auto iter_samples = initial_samples * iter_multiplicity;
      auto next_iter_samples = initial_samples * next_iter_multiplicity;
      if (current_samples + iter_samples + next_iter_samples > total_samples) {
        iter_samples = total_samples - current_samples;
        is_final_iteration = true;
      }

      auto downscale = psl::max(psl::sqrt(float(area(image_size)) / iter_samples), 1.0f);
      auto iter_image_size = vec2i(image_size / downscale);
      auto iter_spp = psl::max<int>(iter_samples / area(iter_image_size), 1);
      iter_samples = area(iter_image_size) * iter_spp;
      auto prime_hit = full_iter >= prime_index * prime_index;
      iterations.emplace_back(iter_image_size, current_sample_index, iter_spp,
                              double(current_samples) / total_samples,
                              double(current_samples + iter_samples) / total_samples, iter,
                              full_iter, is_final_iteration, prime_hit);
      current_sample_index += iter_spp;
      current_samples += iter_samples;

      if (prime_hit)
        prime_index++;
      if (iter_image_size == image_size)
        full_iter++;
    }
  }

  psl::optional<Iteration> next() {
    if (iterations.size()) {
      auto iteration = iterations.front();
      iterations.pop_front();
      return iteration;
    } else {
      return psl::nullopt;
    }
  }

private:
  vec2i image_size;
  size_t total_samples;
  psl::vector<Iteration> iterations;
};

struct OutlierRejectedVariance {
  OutlierRejectedVariance(vec2i image_size) : image_size(image_size), variances(area(image_size)) {
  }

  void set(vec2i p, vec3 sample, vec3 ref, vec3 min_relative) {
    variances[p.x + p.y * image_size.x] = psl::sqr((sample - ref) / max(ref, min_relative));
  }

  vec3 compute(float rejection_fraction) {
    std::sort(variances.begin(), variances.end(),
              [](vec3 a, vec3 b) { return average(a) < average(b); });

    auto N = area(image_size) * (1 - rejection_fraction);
    return psl::mean<vec3d>(psl::trim(variances, 0, N));
  }

private:
  vec2i image_size;
  psl::vector<vec3> variances;
};

struct GuidedPathIntegrator::Vertex {
  Vertex(int length, vec3 throughput, vec3 n, vec3 p, float pdf, bool is_delta = false)
      : length(length), throughput(throughput), n(n), p(p), pdf(pdf), is_delta(is_delta) {
  }
  static Vertex first_vertex(vec3 I) {
    return Vertex(0, vec3(1.0f) / I, vec3(0), vec3(0), 0.0f, true);
  }
  int length;
  vec3 throughput;
  vec3 n;
  vec3 p;
  float pdf;
  bool is_delta;
};

static auto cost_to_var = float(0.0f);

void GuidedPathIntegrator::render(Scene& scene) {
  RTIntegrator::render(scene);
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("`GuidedPathIntegrator` doesn't support `Plane`, please use `Rect` or `Disk` instead");

  light_sampler.build(&scene);
  auto& film = scene.camera.film();
  auto debug_film = film;

  Profiler _("[GuidedPath]Render");

  // auto initial_samples = area(film.size()) / 4;
  auto initial_samples = area(film.size());
  auto spatial_k = 8000;
  guide = SpatialTree(scene.get_aabb(), initial_samples, spatial_k);
  auto spatial_ratio = psl::max<float>(spatial_k / psl::pow<float>(initial_samples, 0.4f), 1.0f);
  auto iteration_scheme = IterativeScheme(film.size(), initial_samples, spp, 3);

  // For denoising later
  auto albedo = Array2d3f(film.size());
  auto normal = Array2d3f(film.size());
  auto position = Array2d3f(film.size());
  parallel_for(film.size(), [&](vec2i p) {
    auto ray = scene.camera.gen_ray((p + vec2(0.5f)) / film.size(), vec2(0.5f));
    if (auto it = Interaction(); intersect(ray, it)) {
      albedo[p] = it.material()->albedo({it.p, it.n, it.uv});
      normal[p] = it.n;
      position[p] = it.p;
    }
  });

  auto acc_I = Array2d3f(film.size());
  auto acc_weight = 0.0f;
  auto I = Array2d3f(film.size());
  auto I_estimate = Array2d3f(film.size());
  auto I_estimate_populated = false;
  auto or_variance = OutlierRejectedVariance(film.size());

  set_progress(0);
  while (auto iter = iteration_scheme.next()) {
    collect_radiance_sample = !iter->is_final;
    use_learned_ratio = iter->number > 2 ? 0.5f : 0.0f;

    auto costs = psl::vector<double>(n_threads());
    parallel_for(iter->image_size, [&](vec2i p) {
      auto& sampler = samplers[threadIdx].start_pixel(p, iter->sample_index);
      auto Ie = I_estimate[p];
      auto L = vec3(0.0f);
      auto cost_total = 0.0f;
      for (int si = 0; si < iter->spp; si++) {
        auto ray = scene.camera.gen_ray((p + sampler.get2d()) / iter->image_size, sampler.get2d());
        auto stats = Stats();
        auto [Li, cost] =
            radiance(scene, ray, sampler, Vertex::first_vertex(max(Ie, vec3(1e-2f))), stats);
        debug_film.add_sample(p, stats.value);
        L += Li;
        cost_total += cost;
        sampler.start_next_sample();
      }
      if (iter->image_size == film.size()) {
        L /= iter->spp;
        I[p] = L;
        or_variance.set(p, L, Ie, vec3(0.01f));
        costs[threadIdx] += cost_total / iter->spp;
      }
      if (p.x % 64 == 0)
        set_progress(psl::lerp(float(p.x + p.y * iter->image_size.x) / area(iter->image_size),
                               iter->progress_start, iter->progress_end));
    });
    auto cost = psl::sum<double>(costs) / area(iter->image_size);

    if (iter->image_size == film.size()) {
      if (!psl::exchange(I_estimate_populated, true)) {
        denoise(DenoiseQuality::Medium, I_estimate, I, albedo, normal);
        parallel_for(film.size(),
                     [&](vec2i p) { or_variance.set(p, I[p], I_estimate[p], vec3(0.01f)); });
      }
      auto variance = or_variance.compute(0.00001f) * iter->spp;  // average variance over samples
      cost_to_var = cost / psl::max<float>(average(variance), epsilon);
      Log("cost: ", (float)cost, " variance: ", (float)average(variance),
          " efficiency: ", 100 / float(cost * average(variance)));
      auto weight = iter->spp / psl::max<float>(average(variance), epsilon);
      combine_inplace(acc_I, I, acc_weight, weight);
      acc_weight += weight;
      if (iter->prime_hit)
        denoise(DenoiseQuality::Medium, I_estimate, acc_I, albedo, normal);

      if (!iter->is_final) {
        guide.refine(spatial_ratio * psl::pow<float>(iter->spp * area(iter->image_size), 0.4f));
        guide.for_each_bin([&](auto& b) {
          if (b.n > 200) {
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
            b.valid = true;
          } else {
            b.valid = false;
          }
        });
      }
    }
  }

  film.pixels = Array2d4f::from(acc_I);
  // film = debug_film;

  // parallel_for(film.size(), [&](vec2i p) {
  //   auto& leaf = guide.traverse(position[{172, 540 - 228}]);
  //   auto w = uniform_sphere(vec2(p) / film.size());
  //   film[p] = vec4(color_map(leaf.flux_density(w)));
  // });

  set_progress(1.0f);
}

GuidedPathIntegrator::RadianceResult GuidedPathIntegrator::radiance(Scene& scene, Ray ray,
                                                                    Sampler& sampler, Vertex pv,
                                                                    Stats& stats) {
  auto result = RadianceResult();
  auto& Lo = result.Lo;
  auto wi = -ray.d;

  auto it = Interaction();
  result.cost += 1;
  if (!intersect(ray, it)) {
    if (scene.env_light && pv.length == 0)
      Lo += scene.env_light->color(ray.d);
    return result;
  }

  if (it.material()->is<EmissiveMaterial>()) {
    if (pv.length == 0)
      Lo += it.material()->le({it, wi});
    return result;
  }

  if (pv.length + 1 >= max_path_length)
    return result;

  auto& leaf = guide.traverse(it.p);
  auto prob_a = it.material()->is_delta() ? 0.0f : use_learned_ratio;
  if (prob_a > 0.0f)
    prob_a = psl::clamp(leaf.strategy_a_prob(), 0.1f, 0.9f);
  auto select_guide = prob_a == 1.0f || (prob_a > 0.0f && sampler.get1d() < prob_a);

  auto& bin = leaf.bin_of(wi);
  auto n = 1.0f;
  auto ni = 1;
  if (bin.valid) {
    n = bin.splitting_factor(pv.throughput, cost_to_var);
    n = psl::clamp(n, 0.05f, 20.0f);
    if (psl::fract(n) == 0.0f)
      ni = n;
    else
      ni = int(psl::floor(n)) + (sampler.get1d() < psl::fract(n) ? 1 : 0);
    if (pv.length == 0)
      stats.value = bin.estimate / bin.n;
    // stats.value = color_map(n / 10.0f);
  }

  auto cost_total = 0;
  auto estimate = vec3(0.0f);
  auto moment2 = vec3(0.0f);

  for (int i = 0; i < ni; i++) {
    auto lo = vec3(0.0f);
    if (!it.material()->is_delta())
      if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
        cost_total += 1;
        if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
          auto cosine = absdot(ls->wo, it.n);
          if (ls->light->is_delta()) {
            auto f = it.material()->f({it, wi, ls->wo});
            lo += ls->le * cosine * f / ls->pdf;
          } else {
            auto [f, bsdf_pdf] = it.material()->f_pdf({it, wi, ls->wo});
            auto mis = balance_heuristic(ls->pdf, bsdf_pdf);
            lo += ls->le * cosine * f / ls->pdf * mis;
          }
        }
      }
    if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
      auto nray = it.spawn_ray(bs->wo);
      auto nit = Interaction();
      cost_total += 1;
      if (intersect(nray, nit)) {
        if (nit.material()->is<EmissiveMaterial>()) {
          auto cosine = absdot(bs->wo, it.n);
          auto mis = 1.0f;
          if (!it.material()->is_delta()) {
            auto light_pdf = light_sampler.pdf(nit.geometry, nit, nray, it.n);
            mis = balance_heuristic(bs->pdf, light_pdf);
          }
          lo += nit.material()->le({nit, -bs->wo}) * cosine * bs->f / bs->pdf * mis;
        }
      } else if (scene.env_light) {
        auto cosine = absdot(bs->wo, it.n);
        auto mis = 1.0f;
        if (!it.material()->is_delta()) {
          auto light_pdf = scene.env_light->pdf(it.n, bs->wo);
          mis = balance_heuristic(bs->pdf, light_pdf);
        }
        lo += scene.env_light->color(bs->wo) * cosine * bs->f / bs->pdf * mis;
      }
    }

    if (select_guide) {
      if (auto gs = leaf.sample(sampler.get2d())) {
        auto cosine = absdot(gs->wo, it.n);
        auto mec = MaterialEvalCtx(it, wi, gs->wo);
        auto f = it.material()->f(mec);
        auto nv =
            Vertex(pv.length + 1, pv.throughput / n * f / gs->pdf * cosine, it.n, it.p, gs->pdf);
        auto [Li, cost] = radiance(scene, it.spawn_ray(gs->wo), sampler, nv, stats);
        auto mis_indirect = balance_heuristic(prob_a, gs->pdf, 1 - prob_a, it.material()->pdf(mec));
        lo += Li * cosine * f / gs->pdf * mis_indirect / prob_a;
        cost_total += cost;
        if (collect_radiance_sample)
          guide.add_sample(leaf, it.p,
                           RadianceSample(gs->wo, luminance(Li / gs->pdf * mis_indirect / prob_a),
                                          cosine * luminance(Li * f), psl::nullopt),
                           sampler.get3d(), {});
      }
    } else {
      if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
        auto cosine = absdot(bs->wo, it.n);
        auto nv = Vertex(pv.length + 1, pv.throughput / n * bs->f / bs->pdf * cosine, it.n, it.p,
                         bs->pdf, it.material()->is_delta());
        auto [Li, cost] = radiance(scene, it.spawn_ray(bs->wo), sampler, nv, stats);
        auto mis_indirect = balance_heuristic(1 - prob_a, bs->pdf, prob_a, leaf.pdf(bs->wo));
        lo += Li * cosine * bs->f / bs->pdf * mis_indirect / (1 - prob_a);
        cost_total += cost;
        if (collect_radiance_sample && !it.material()->is_delta())
          guide.add_sample(
              leaf, it.p,
              RadianceSample(bs->wo, luminance(Li / bs->pdf * mis_indirect / (1 - prob_a)),
                             psl::nullopt, cosine * luminance(Li * bs->f)),
              sampler.get3d(), {});
      }
    }
    estimate += lo;
    moment2 += lo * lo;
  }

  Lo += estimate / (n < 1 ? n : ni);
  if (ni != 0) {
    if (collect_radiance_sample) {
      bin.cost += cost_total;
      bin.estimate += estimate;
      bin.moment2 += moment2;
      bin.n += ni;
    }
    result.cost += cost_total;
  }

  return result;
}

}  // namespace pine