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
    bool is_final;
    bool prime_hit;
  };

  IterativeScheme(vec2i image_size, size_t initial_samples, int spp, int batch_size)
      : image_size(image_size), total_samples(size_t(area(image_size)) * spp) {
    auto current_samples = size_t(0);
    auto current_sample_index = 0;
    auto iter_multiplicity = 1;
    int prime_index = 1;

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
                              is_final_iteration, prime_hit);
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

void GuidedPathIntegrator::render(Scene& scene) {
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("GuidedPathIntegrator doesn't support `Plane`, please use `Rect` or `Disk` instead");

  accel.build(&scene);
  light_sampler.build(&scene);
  auto& film = scene.camera.film();

  Profiler _("[GuidedPath]Render");

  // auto initial_samples = size_t(1024 * 16);
  auto initial_samples = (size_t)area(film.size());
  auto spatial_k = 4000;
  guide = SpatialTree(scene.get_aabb(), initial_samples, spatial_k);
  auto iteration_scheme = IterativeScheme(film.size(), initial_samples, spp, 4);

  // For denoising later
  auto albedo = Array2d3f(film.size());
  auto normal = Array2d3f(film.size());
  parallel_for(film.size(), [&](vec2i p) {
    auto ray = scene.camera.gen_ray((p + vec2(0.5f)) / film.size(), vec2(0.5f));
    if (auto it = Interaction(); intersect(ray, it)) {
      albedo[p] = it.material()->albedo({it.p, it.n, it.uv});
      normal[p] = it.n;
    }
  });

  auto acc_I = Array2d3f(film.size());
  auto acc_weight = 0.0;
  auto I = Array2d3f(film.size());
  auto I_estimate = Array2d3f(film.size());
  auto I_estimate_populated = false;
  auto vars = psl::vector<vec3>(area(film.size()));

  collect_radiance_sample = true;
  use_learned_ratio = 0.0f;
  set_progress(0);
  while (auto iter = iteration_scheme.next()) {
    if (iter->is_final)
      collect_radiance_sample = false;
    use_learned_ratio = iter->number > 2 || iter->image_size == film.size() ? 0.5f : 0.0f;

    parallel_for(iter->image_size, [&](vec2i p) {
      auto& sampler = samplers[threadIdx].start_pixel(p, iter->sample_index);
      auto Ie = I_estimate[p];
      auto L = vec3(0.0f);
      for (int si = 0; si < iter->spp; si++) {
        auto ray = scene.camera.gen_ray((p + sampler.get2d()) / iter->image_size, sampler.get2d());
        L += radiance(scene, ray, sampler, Vertex::first_vertex());
        sampler.start_next_sample();
      }
      L /= iter->spp;
      I[p] = L;
      vars[p.x + film.size().x * p.y] = psl::sqr((L - Ie) / max(Ie, vec3(1e-2f)));

      if (p.x % 64 == 0)
        set_progress(psl::lerp(float(p.x + p.y * iter->image_size.x) / area(iter->image_size),
                               iter->progress_start, iter->progress_end));
    });

    if (iter->image_size == film.size()) {
      if (!I_estimate_populated) {
        denoise(DenoiseQuality::Medium, I_estimate, I, albedo, normal);
        parallel_for(film.size(), [&](vec2i p) {
          auto Ie = I_estimate[p];
          vars[p.x + film.size().x * p.y] = psl::sqr((I[p] - Ie) / max(Ie, vec3(1e-2f)));
        });
        I_estimate_populated = true;
      }
      std::sort(vars.begin(), vars.end(), [](vec3 a, vec3 b) { return average(a) < average(b); });
      auto N = area(film.size()) * (1 - 0.00001f);
      auto variance = average(psl::mean<vec3d>(psl::trim(vars, 0, N)));
      Log("Variance: ", variance);
      auto weight = 1.0f / variance;
      combine_inplace(acc_I, I, acc_weight, weight);
      acc_weight += weight;
      if (iter->prime_hit)
        denoise(DenoiseQuality::Medium, I_estimate, acc_I, albedo, normal);
    }

    if (!iter->is_final)
      guide.refine(spatial_k * psl::sqrt<float>(iter->spp));
  }

  film.pixels = Array2d4f::from(acc_I);
  set_progress(1.0f);
}

vec3 GuidedPathIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler, Vertex pv) {
  auto Lo = vec3(0.0f);
  auto wi = -ray.d;

  auto it = Interaction();
  if (!intersect(ray, it)) {
    if (scene.env_light && pv.length == 0)
      Lo += scene.env_light->color(ray.d);
    return Lo;
  }

  if (it.material()->is<EmissiveMaterial>()) {
    if (pv.length == 0)
      Lo += it.material()->le({it, wi});
    return Lo;
  }

  if (pv.length + 1 >= max_path_length)
    return Lo;

  auto& leaf = guide.traverse(it.p);
  auto prob_a = it.material()->is_delta() ? 0.0f : use_learned_ratio;
  if (prob_a > 0.0f)
    prob_a = psl::clamp(leaf.strategy_a_prob(), 0.1f, 0.9f);
  auto select_guide = prob_a == 1.0f || (prob_a > 0.0f && sampler.get1d() < prob_a);

  if (!it.material()->is_delta())
    if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
      if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
        auto cosine = absdot(ls->wo, it.n);
        if (ls->light->is_delta()) {
          auto f = it.material()->f({it, wi, ls->wo});
          Lo += ls->le * cosine * f / ls->pdf;
        } else {
          auto [f, bsdf_pdf] = it.material()->f_pdf({it, wi, ls->wo});
          auto mis = balance_heuristic(ls->pdf, bsdf_pdf);
          Lo += ls->le * cosine * f / ls->pdf * mis;
        }
      }
    }
  if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
    auto nray = it.spawn_ray(bs->wo);
    auto nit = Interaction();
    if (intersect(nray, nit)) {
      if (nit.material()->is<EmissiveMaterial>()) {
        auto cosine = absdot(bs->wo, it.n);
        auto mis = 1.0f;
        if (!it.material()->is_delta()) {
          auto light_pdf = light_sampler.pdf(nit.geometry, nit, nray, it.n);
          mis = balance_heuristic(bs->pdf, light_pdf);
        }
        Lo += nit.material()->le({nit, -bs->wo}) * cosine * bs->f / bs->pdf * mis;
      }
    } else if (scene.env_light) {
      auto cosine = absdot(bs->wo, it.n);
      auto mis = 1.0f;
      if (!it.material()->is_delta()) {
        auto light_pdf = scene.env_light->pdf(it.n, bs->wo);
        mis = balance_heuristic(bs->pdf, light_pdf);
      }
      Lo += scene.env_light->color(bs->wo) * cosine * bs->f / bs->pdf * mis;
    }
  }

  if (select_guide) {
    if (auto gs = leaf.sample(sampler.get2d())) {
      auto nv = Vertex(pv.length + 1, it.n, it.p, gs->pdf);
      auto Li = radiance(scene, it.spawn_ray(gs->wo), sampler, nv);
      auto cosine = absdot(gs->wo, it.n);
      auto mec = MaterialEvalCtx(it, wi, gs->wo);
      auto f = it.material()->f(mec);
      auto mis_indirect = balance_heuristic(prob_a, gs->pdf, 1 - prob_a, it.material()->pdf(mec));
      Lo += Li * cosine * f / gs->pdf * mis_indirect / prob_a;
      if (collect_radiance_sample)
        guide.add_sample(leaf, it.p,
                         RadianceSample(gs->wo, luminance(Li / gs->pdf * mis_indirect / prob_a),
                                        cosine * luminance(Li * f), psl::nullopt),
                         sampler.get3d(), {});
    }
  } else {
    if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
      auto nv = Vertex(pv.length + 1, it.n, it.p, bs->pdf, it.material()->is_delta());
      auto Li = radiance(scene, it.spawn_ray(bs->wo), sampler, nv);
      auto cosine = absdot(bs->wo, it.n);
      auto mis_indirect = balance_heuristic(1 - prob_a, bs->pdf, prob_a, leaf.pdf(bs->wo));
      Lo += Li * cosine * bs->f / bs->pdf * mis_indirect / (1 - prob_a);
      if (collect_radiance_sample && !it.material()->is_delta())
        guide.add_sample(
            leaf, it.p,
            RadianceSample(bs->wo, luminance(Li / bs->pdf * mis_indirect / (1 - prob_a)),
                           psl::nullopt, cosine * luminance(Li * bs->f)),
            sampler.get3d(), {});
    }
  }

  return Lo;
}

}  // namespace pine