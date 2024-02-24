#include <pine/impl/integrator/spatial_tree.h>
#include <pine/impl/integrator/guidedpath.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

namespace pine {

static SpatialNodeRoot guide;

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

  Profiler _("[Integrator]Rendering");

  auto total_pixels = size_t(area(film.size()));
  auto total_samples = total_pixels * samples_per_pixel;
  auto initial_samples = size_t(1024 * 16);
  auto current_sample_index = 0;
  auto current_samples = size_t(0);
  auto accumulated_image = Array2d3f(film.size());
  auto accumulated_weight = 0.0;
  auto image = Array2d3f(film.size());
  auto squared_image = Array2d3f(film.size());
  guide = SpatialNodeRoot(scene.get_aabb(), QuadTree());
  guide.initial_refinement(initial_samples);

  for (int iteration = 0; current_samples < total_samples; iteration++) {
    auto iter_samples = initial_samples * (1 << iteration);
    auto downscale = psl::max(psl::sqrt(float(total_pixels) / iter_samples), 1.0f);
    auto iter_image_size = vec2i(film.size() / downscale);
    auto iter_spp = psl::max<int>(iter_samples / area(iter_image_size), 1);
    iter_samples = area(iter_image_size) * iter_spp;
    current_samples += iter_samples;
  }
  total_samples = psl::exchange(current_samples, 0);

  auto on_final_rendering = false;
  collect_radiance_sample = true;
  use_learned_ratio = 0.0f;
  set_progress(0);
  for (int iteration = 0; current_samples < total_samples; iteration++) {
    auto iter_samples = initial_samples * (1 << iteration);
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

    auto primary_ratio = this->primary_ratio;
    auto primary_spp = iter_spp;
    auto secondary_spp = 1;
    while ((primary_ratio % 2 == 0) && (primary_spp % 2 == 0)) {
      primary_ratio /= 2;
      primary_spp /= 2;
      secondary_spp *= 2;
    }
    for (int i = 0; i < primary_spp; i++) {
      Atomic<int64_t> max_index = 0;
      parallel_for(iter_image_size, [&](vec2i p) {
        auto& sampler = samplers[threadIdx].start_pixel(p, current_sample_index);
        auto p_film = vec2(p + sampler.get2d()) / iter_image_size;
        auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
        auto it = Interaction();
        auto is_hit = intersect(ray, it);
        for (int i = 0; i < secondary_spp; i++) {
          auto [L, _] = radiance(scene, ray, it, is_hit, sampler, Vertex::first_vertex());
          if (iter_image_size == film.size()) {
            image[p] += L;
            squared_image[p] += L * L;
          }
          sampler.start_next_sample();
        }
        if (p.x == 0) {
          max_index = psl::max<int64_t>(max_index, p.x + p.y * iter_image_size.x);
          set_progress(double(current_samples + max_index * secondary_spp) / total_samples);
        }
      });
      current_sample_index += secondary_spp;
      current_samples += secondary_spp * area(iter_image_size);
    }
    if (iter_spp > 1) {
      image /= iter_spp;
      squared_image /= iter_spp;
    }

    if (iter_spp > 1) {
      auto variance = 0.0;
      for_2d(iter_image_size, [&](vec2i p) {
        auto sc = squared_image[p];
        auto c = image[p];
        auto local_variance = luminance(abs(sc - c * c));
        variance += double(local_variance);
      });
      variance = variance / (area(iter_image_size) * iter_spp);
      auto weight = 1.0 / variance;
      accumulated_image = combine(accumulated_image, image, accumulated_weight, weight);
      accumulated_weight += weight;
      Debug("Variance: ", float(variance));
    }

    if (!on_final_rendering)
      guide.refine(iteration);
    image.set_to_zero();
    squared_image.set_to_zero();
  }

  for_2d(film.size(), [&](vec2i p) { film.add_sample_no_acc(p, accumulated_image[p]); });
  set_progress(1.0f);
}

GuidedPathIntegrator::RadianceResult GuidedPathIntegrator::radiance(Scene& scene, Ray ray,
                                                                    Interaction it, bool is_hit,
                                                                    Sampler& sampler, Vertex pv) {
  auto Lo = vec3(0.0f);
  auto wi = -ray.d;

  if (pv.length > 0)
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
      auto light_pdf = it.geometry->pdf(it, ray, pv.n);
      auto mis_term = balance_heuristic(pv.pdf, light_pdf);
      return {Lo, mis_term};
    }
    return {Lo, psl::nullopt};
  }

  if (pv.length + 1 >= max_path_length)
    return {Lo, psl::nullopt};

  auto& leaf = guide.traverse(it.p);

  auto prob_a = it.material()->is_delta() ? 0.0f : use_learned_ratio;
  auto select_guide = prob_a > 0.0f && sampler.get1d() < prob_a;

  if (!it.material()->is_delta())
    if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
      if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
        auto cosine = absdot(ls->wo, it.n);
        if (select_guide) {
          auto f = it.material()->f({it, wi, ls->wo});
          auto guide_pdf = leaf.pdf(ls->wo);
          auto mis = balance_heuristic(ls->pdf, guide_pdf);
          if (ls->light->is_delta())
            mis = 1;
          Lo += ls->le * cosine * f / ls->pdf * mis;
        } else {
          auto [f, bsdf_pdf] = it.material()->f_pdf({it, wi, ls->wo});
          auto mis = balance_heuristic(ls->pdf, bsdf_pdf);
          if (ls->light->is_delta())
            mis = 1;
          Lo += ls->le * cosine * f / ls->pdf * mis;
        }
      }
    }

  if (select_guide) {
    if (auto gs = leaf.sample(sampler.get2d())) {
      auto nv = Vertex(pv.length + 1, it.n, it.p, gs->pdf);
      auto [Li, mis_direct] = radiance(scene, it.spawn_ray(gs->wo), {}, {}, sampler, nv);
      auto cosine = absdot(gs->wo, it.n);
      auto mec = MaterialEvalCtx(it, wi, gs->wo);
      auto f = it.material()->f(mec);
      if (mis_direct) {
      } else {
        auto mis_indirect =
            prob_a == 1.0f ? 1.0f : balance_heuristic(gs->pdf, it.material()->pdf(mec));
        Lo += Li * cosine * f / gs->pdf * mis_indirect / prob_a;
        if (collect_radiance_sample)
          guide.add_sample(leaf, it.p, RadianceSample(gs->wo, Li / gs->pdf * mis_indirect / prob_a),
                           sampler.get3d(), sampler.get2d());
      }
    }
  } else {
    if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
      auto nv = Vertex(pv.length + 1, it.n, it.p, bs->pdf, it.material()->is_delta());
      auto [Li, mis_direct] = radiance(scene, it.spawn_ray(bs->wo), {}, {}, sampler, nv);
      auto cosine = absdot(bs->wo, it.n);
      if (mis_direct) {
        Lo += Li * cosine * bs->f / bs->pdf * (*mis_direct);
      } else {
        auto mis_indirect = prob_a == 0.0f ? 1.0f : balance_heuristic(bs->pdf, leaf.pdf(bs->wo));
        Lo += Li * cosine * bs->f / bs->pdf * mis_indirect / (1 - prob_a);
        if (collect_radiance_sample && !it.material()->is_delta())
          guide.add_sample(leaf, it.p,
                           RadianceSample(bs->wo, Li / bs->pdf * mis_indirect / (1 - prob_a)),
                           sampler.get3d(), sampler.get2d());
      }
    }
  }

  return {Lo, psl::nullopt};
}

}  // namespace pine