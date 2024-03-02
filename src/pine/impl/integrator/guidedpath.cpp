#include <pine/impl/integrator/spatial_tree.h>
#include <pine/impl/integrator/guidedpath.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

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

void GuidedPathIntegrator::render(Scene& scene) {
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("GuidedPathIntegrator doesn't support `Plane`, please use `Rect` or `Disk` instead");

  accel.build(&scene);
  light_sampler.build(&scene);
  auto& film = scene.camera.film();
  film.clear();

  Profiler _("[Integrator]Rendering");

  auto total_pixels = size_t(area(film.size()));
  auto total_samples = total_pixels * spp;
  auto initial_samples = size_t(1024 * 16);
  auto current_sample_index = 0;
  auto current_samples = size_t(0);
  auto accumulated_image = Array2d3f(film.size());
  auto accumulated_weight = 0.0;
  auto image = Array2d3f(film.size());
  auto squared_image = Array2d3f(film.size());
  auto spatial_k = 1000;
  guide = SpatialTree(scene.get_aabb(), initial_samples, spatial_k);

  auto iter_samples = size_t(0);
  for (int iter = 0; current_samples < total_samples; iter++) {
    iter_samples = initial_samples * (1 << iter);
    auto downscale = psl::max(psl::sqrt(float(total_pixels) / iter_samples), 1.0f);
    auto iter_image_size = vec2i(film.size() / downscale);
    auto iter_spp = psl::max<int>(iter_samples / area(iter_image_size), 1);
    iter_samples = area(iter_image_size) * iter_spp;
    current_samples += iter_samples;
  }
  if (current_samples > total_samples)
    current_samples -= iter_samples;
  total_samples = psl::exchange(current_samples, 0);
  Log("Effective spp: ", total_samples / area(film.size()));

  auto on_final_rendering = false;
  collect_radiance_sample = true;
  use_learned_ratio = 0.0f;
  set_progress(0);
  for (int iter = 0; current_samples < total_samples; iter++) {
    auto iter_samples = initial_samples * (1 << iter);
    auto downscale = psl::max(psl::sqrt(float(total_pixels) / iter_samples), 1.0f);
    auto iter_image_size = vec2i(film.size() / downscale);
    auto iter_spp = psl::max<int>(iter_samples / area(iter_image_size), 1);
    iter_samples = area(iter_image_size) * iter_spp;

    if (current_samples + iter_samples >= total_samples) {
      on_final_rendering = true;
      iter_image_size = film.size();
      iter_spp = psl::max(iter_spp, 2);
      collect_radiance_sample = false;
    }
    use_learned_ratio = iter_spp > 1 ? 0.5f : 0.0f;

    for (int i = 0; i < iter_spp; i++) {
      Atomic<int64_t> max_index = 0;
      parallel_for(iter_image_size, [&](vec2i p) {
        auto& sampler = samplers[threadIdx].start_pixel(p, current_sample_index);
        auto p_film = vec2(p + sampler.get2d()) / iter_image_size;
        auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
        auto L = radiance(scene, ray, sampler, Vertex::first_vertex());
        if (iter_image_size == film.size()) {
          image[p] += L;
          squared_image[p] += L * L;
        }
        sampler.start_next_sample();
        if (p.x == 0) {
          max_index = psl::max<int64_t>(max_index, p.x + p.y * iter_image_size.x);
          set_progress(double(current_samples + max_index) / total_samples);
        }
      });
      current_sample_index += 1;
      current_samples += area(iter_image_size);
    }

    if (iter_spp > 1) {
      image /= iter_spp;
      squared_image /= iter_spp;
      auto variance = 0.0;
      for_2d(iter_image_size, [&](vec2i p) {
        auto sc = squared_image[p];
        auto c = image[p];
        auto local_variance = psl::min(luminance(abs(sc - c * c)), 100000.0f);
        variance += double(local_variance);
      });
      variance = variance / (area(iter_image_size) * iter_spp);
      auto weight = 1.0 / variance;
      accumulated_image = combine(accumulated_image, image, accumulated_weight, weight);
      accumulated_weight += weight;
      // if (!collect_radiance_sample)
      //   accumulated_image = image;
      Debug("Variance: ", float(variance));
    }

    if (!on_final_rendering)
      guide.refine(spatial_k * psl::sqrt<float>(1 << iter));
    image.set_to_zero();
    squared_image.set_to_zero();
  }

  for_2d(film.size(), [&](vec2i p) { film[p] = vec4(accumulated_image[p], 1.0f); });
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
  // if (!collect_radiance_sample)
  //   return color_map(leaf.strategy_a_prob());
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