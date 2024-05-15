#include <pine/impl/integrator/spatial_tree.h>
#include <pine/impl/integrator/guidedpath.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/denoise.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>
#include <pine/core/gui.h>

#include <algorithm>

namespace pine {
static SpatialTree guide;

GuidedPathIntegrator::GuidedPathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                                           int max_path_length)
    : RTIntegrator{MOVE(accel), MOVE(sampler)},
      light_sampler{MOVE(light_sampler)},
      max_path_length{max_path_length} {
  if (max_path_length <= 0)
    Fatal("`GuidedPathIntegrator` expect `max_path_length` to be positive, get", max_path_length);
}

struct IterativeScheme {
  struct Iteration {
    int number;
    int sample_index;
    int spp;
    bool is_final;
    bool prime_hit;
    float progress_start, progress_end;
  };

  IterativeScheme(int spp, int batch_size) {
    auto current_spp = 0;
    auto iter_spp = 2;
    auto prime_index = 1;

    auto is_final_iteration = false;
    for (int iter = 0; !is_final_iteration; iter++) {
      auto next_iter_spp = iter_spp;
      if (iter % batch_size == batch_size - 1)
        next_iter_spp *= 2;

      auto remaining_spp = spp - current_spp;
      if (iter_spp + next_iter_spp / 2 > remaining_spp) {
        iter_spp = remaining_spp;
        is_final_iteration = true;
      }

      auto prime_hit = iter >= prime_index * prime_index;
      iterations.emplace_back(iter, current_spp, iter_spp, is_final_iteration, prime_hit,
                              float(current_spp) / spp, float(current_spp + iter_spp) / spp);
      if (prime_hit)
        prime_index++;
      current_spp += psl::exchange(iter_spp, next_iter_spp);
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
  psl::vector<Iteration> iterations;
};

struct OutlierRejectedVariance {
  OutlierRejectedVariance(vec2i image_size) : image_size(image_size), variances(area(image_size)) {
  }

  void set(vec2i p, vec3 var) {
    variances[p.x + p.y * image_size.x] = var;
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
  Vertex(int length, float pdf, bool is_delta = false)
      : length(length), pdf(pdf), is_delta(is_delta) {
  }
  static Vertex first_vertex() {
    return Vertex(0, 0.0f, true);
  }
  int length;
  float pdf;
  bool is_delta;
};

void GuidedPathIntegrator::render(Scene& scene) {
  RTIntegrator::render(scene);
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("`GuidedPathIntegrator` doesn't support `Plane`, please use `Rect` or `Disk` instead");

  light_sampler.build(&scene);
  auto& film = scene.camera.film();

  Profiler _("[GuidedPath]Render");

  auto initial_samples = area(film.size());
  auto spatial_k = 4000;
  guide = SpatialTree(scene.get_aabb(), initial_samples, spatial_k);
  auto spatial_ratio = psl::max<float>(spatial_k / psl::sqrt<float>(initial_samples), 1.0f);
  auto iteration_scheme = IterativeScheme(spp, 1);

  auto acc_I = Array2d3f(film.size());
  auto acc_weight = 0.0f;
  auto I = Array2d3f(film.size());
  auto or_variance = OutlierRejectedVariance(film.size());

  set_progress(0);
  while (auto iter = iteration_scheme.next()) {
    collect_radiance_sample = !iter->is_final;
    use_learned_ratio = iter->number > 0 ? 0.5f : 0.0f;

    parallel_for(film.size(), [&](vec2i p) {
      auto& sampler = samplers[threadIdx].start_pixel(p, iter->sample_index);
      auto L = vec3(0.0f);
      auto L_squared = vec3(0.0f);
      for (int si = 0; si < iter->spp; si++, sampler.start_next_sample()) {
        auto ray = scene.camera.gen_ray((p + sampler.get2d()) / film.size(), sampler.get2d());
        auto Li = radiance(scene, ray, sampler, Vertex::first_vertex());
        L += Li;
        L_squared += Li * Li;
      }
      I[p] = L / iter->spp;
      if (iter->spp > 1)
        or_variance.set(p, (L_squared - L * L / iter->spp) / (iter->spp - 1) / iter->spp);
      if (p.x % 64 == 0)
        set_progress(psl::lerp(float(p.x + p.y * film.size().x) / area(film.size()),
                               iter->progress_start, iter->progress_end));
    });

    auto variance = or_variance.compute(0.00001f);
    Debug("variance: ", (float)average(variance));
    auto weight = 1.0f / psl::max<float>(average(variance), epsilon);
    combine_inplace(acc_I, I, acc_weight, weight);
    acc_weight += weight;

    if (!iter->is_final)
      guide.refine(spatial_ratio * psl::sqrt<float>(iter->spp * area(film.size())));
  }

  film.pixels = Array2d4f::from(acc_I);
  set_progress(1.0f);
}

vec3 GuidedPathIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler, Vertex pv) {
  auto Lo = vec3(0.0f);
  auto wi = -ray.d;

  auto [mit, it] = intersect_tr(ray, sampler);
  if (mit) {
    if (pv.length + 1 >= max_path_length)
      return Lo;

    // Sample direct lighting
    if (auto ls = light_sampler.sample(*mit, sampler.get1d(), sampler.get2d())) {
      if (!hit(Ray(mit->p, ls->wo, 0.0f, ls->distance))) {
        auto tr = transmittance(mit->p, ls->wo, ls->distance, sampler);
        auto f = mit->pg.f(wi, ls->wo);
        Lo += ls->le * tr * f / ls->pdf;
      }
    }

    // Sample indirect lighting
    auto& leaf = guide.traverse(mit->p);
    auto prob_a = use_learned_ratio;
    if (with_probability(prob_a, sampler)) {
      if (auto gs = leaf.sample(sampler.get2d())) {
        auto rr = pv.length <= 1 ? 1.0f : psl::max(gs->pdf, 0.05f);
        if (rr >= 1 || sampler.get1d() < rr) {
          auto f = mit->pg.f(wi, gs->wo);
          auto nv = Vertex(pv.length + 1, gs->pdf);
          auto mis = balance_heuristic(prob_a, gs->pdf, 1 - prob_a, mit->pg.pdf(wi, gs->wo));
          auto Li = radiance(scene, Ray(mit->p, gs->wo), sampler, nv);
          Lo += Li * f / gs->pdf * mis / prob_a;
          if (collect_radiance_sample)
            guide.add_sample(leaf, mit->p,
                             RadianceSample(gs->wo, average(Li) / gs->pdf * mis / prob_a),
                             sampler.get3d());
        }
      }
    } else {
      auto ps = mit->pg.sample(wi, sampler.get2d());
      auto rr = pv.length <= 1 ? 1.0f : psl::max(ps.f / ps.pdf, 0.05f);
      if (rr >= 1 || sampler.get1d() < rr) {
        auto nv = Vertex(pv.length + 1, ps.pdf);
        auto mis = balance_heuristic(1 - prob_a, ps.pdf, prob_a, leaf.pdf(ps.wo));
        auto Li = radiance(scene, Ray(mit->p, ps.wo), sampler, nv);
        Lo += Li * (ps.f / ps.pdf * mis / psl::min(1.0f, rr));
        if (collect_radiance_sample)
          guide.add_sample(leaf, mit->p, RadianceSample(ps.wo, average(Li) / ps.pdf * mis / prob_a),
                           sampler.get3d());
      }
    }

    return Lo;
  }

  auto Tr = transmittance(ray.o, ray.d, ray.tmax, sampler);
  if (!it) {
    if (scene.env_light && pv.length == 0)
      Lo += Tr * scene.env_light->color(ray.d);
    return Lo;
  }
  if (it->material().is<EmissiveMaterial>()) {
    if (pv.length == 0)
      Lo += Tr * it->material().le({*it, wi});
    return Lo;
  }
  if (pv.length + 1 >= max_path_length)
    return Lo;

  auto lo = vec3(0.0f);
  // Sample direct lighting
  if (!it->material().is_delta(*it))
    if (auto ls = light_sampler.sample(*it, sampler.get1d(), sampler.get2d())) {
      if (!hit(it->spawn_ray(ls->wo, ls->distance))) {
        auto cosine = absdot(ls->wo, it->n);
        auto tr = transmittance(it->p, ls->wo, ls->distance, sampler);
        if (ls->light->is_delta()) {
          auto f = it->material().f({*it, wi, ls->wo});
          lo += ls->le * tr * cosine * f / ls->pdf;
        } else {
          auto [f, bsdf_pdf] = it->material().f_pdf({*it, wi, ls->wo});
          auto mis = balance_heuristic(ls->pdf, bsdf_pdf);
          lo += ls->le * tr * cosine * f / ls->pdf * mis;
        }
      }
    }
  if (auto bs = it->material().sample({*it, wi, sampler.get1d(), sampler.get2d()})) {
    auto nray = it->spawn_ray(bs->wo);
    if (auto nit = SurfaceInteraction(); intersect(nray, nit)) {
      if (nit.material().is<EmissiveMaterial>()) {
        auto cosine = absdot(bs->wo, it->n);
        auto mis = 1.0f;
        if (!it->material().is_delta(*it)) {
          auto light_pdf = light_sampler.pdf(*it, nit, nray);
          mis = balance_heuristic(bs->pdf, light_pdf);
        }
        lo += nit.material().le({nit, -bs->wo}) * cosine * bs->f / bs->pdf * mis;
      }
    } else if (scene.env_light) {
      auto cosine = absdot(bs->wo, it->n);
      auto mis = 1.0f;
      if (!it->material().is_delta(*it)) {
        auto light_pdf = scene.env_light->pdf(*it, bs->wo);
        mis = balance_heuristic(bs->pdf, light_pdf);
      }
      lo += scene.env_light->color(bs->wo) * cosine * bs->f / bs->pdf * mis;
    }
  }

  // Sample indirect lighting
  auto& leaf = guide.traverse(it->p);
  auto prob_a = it->material().is_delta(*it) ? 0.0f : use_learned_ratio;
  if (with_probability(prob_a, sampler)) {
    if (auto gs = leaf.sample(sampler.get2d())) {
      auto cosine = absdot(gs->wo, it->n);
      auto mec = MaterialEvalCtx(*it, wi, gs->wo);
      auto f = it->material().f(mec);
      auto nv = Vertex(pv.length + 1, gs->pdf);
      auto Li = radiance(scene, it->spawn_ray(gs->wo), sampler, nv);
      auto mis = balance_heuristic(prob_a, gs->pdf, 1 - prob_a, it->material().pdf(mec));
      lo += Li * cosine * f / gs->pdf * mis / prob_a;
      if (collect_radiance_sample)
        guide.add_sample(leaf, it->p, RadianceSample(gs->wo, average(Li) / gs->pdf * mis / prob_a),
                         sampler.get3d());
    }
  } else {
    if (auto bs = it->material().sample({*it, wi, sampler.get1d(), sampler.get2d()})) {
      auto cosine = absdot(bs->wo, it->n);
      auto nv = Vertex(pv.length + 1, bs->pdf, it->material().is_delta(*it));
      auto Li = radiance(scene, it->spawn_ray(bs->wo), sampler, nv);
      auto mis = balance_heuristic(1 - prob_a, bs->pdf, prob_a, leaf.pdf(bs->wo));
      lo += Li * cosine * bs->f / bs->pdf * mis / (1 - prob_a);
      if (collect_radiance_sample && !it->material().is_delta(*it))
        guide.add_sample(leaf, it->p,
                         RadianceSample(bs->wo, average(Li) / bs->pdf * mis / (1 - prob_a)),
                         sampler.get3d());
    }
  }
  Lo += Tr * lo;
  
  return Lo;
}

}  // namespace pine