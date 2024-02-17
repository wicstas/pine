#include <pine/impl/integrator/spatial_tree.h>
#include <pine/impl/integrator/guidedpath.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

#include <tbb/parallel_for.h>

namespace pine {

static SpatialTree guide;

void GuidedPathIntegrator::render(Scene& scene) {
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("GuidedPathIntegrator doesn't support `Plane`, please use `Rect` or `Disk` instead");

  accel.build(&scene);
  light_sampler.build(&scene);
  auto& film = scene.camera.film();
  film.clear();

  Profiler _("Rendering");

  auto total_pixels = size_t(area(film.size()));
  auto learning_samples = total_pixels * samples_per_pixel;
  auto total_samples = double(learning_samples + total_pixels * estimate_samples);
  auto initial_samples = size_t(1024 * 16);
  auto n_iterations = int(psl::ceil(psl::log2(learning_samples / initial_samples + 1.0f)));
  Debug("[GuidedPath]", n_iterations, " learning iterations");

  guide = SpatialTree(scene.get_aabb(), QuadTree());
  guide.initial_refinement(initial_samples * 4);

  auto current_sample_index = 0;
  auto current_samples = size_t(0);

  set_progress(0);
  collect_radiance_sample = true;
  use_learned_ratio = 0.0f;
  for (int iteration = 0; iteration < n_iterations; iteration++) {
    Debug("iter: ", iteration, ", max tree depth: ", guide.max_tree_depth());
    const auto iter_samples = initial_samples * (1 << iteration);
    const auto is_last_iter = iteration + 1 == n_iterations;

    use_learned_ratio = psl::min(psl::sqrt(float(iteration) / n_iterations), 0.8f);
    if (is_last_iter) {
      if (!use_estimate)
        collect_radiance_sample = false;
      use_learned_ratio = 0.8f;
    }

    const auto downscale = psl::min(psl::sqrt(float(iter_samples) / total_pixels), 1.0f);
    const auto iter_image_size = vec2i(film.size() * downscale);
    const auto iter_n_pass = iter_samples / area(iter_image_size);
    for (size_t si = 0; si < iter_n_pass; si++) {
      parallel_for(iter_image_size, [&](vec2i p) {
        auto& sampler = samplers[threadIdx].start_pixel(p, current_sample_index);
        auto p_film = vec2(p + sampler.get2d()) / iter_image_size;
        auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
        auto [L, _] = radiance(scene, ray, sampler, Vertex::first_vertex());
        if (is_last_iter && !use_estimate)
          film.add_sample(p_film * film.size(), L);
      });
      current_sample_index += 1;
      current_samples += area(iter_image_size);
      set_progress(current_samples / total_samples);
    }

    if (!is_last_iter || use_estimate)
      guide.refine(iteration);
    Debug("iter: ", iteration, ", max tree depth: ", guide.max_tree_depth());
  }

  if (use_estimate) {
    collect_radiance_sample = false;
    for (int si = 0; si < estimate_samples; si++) {
      parallel_for(film.size(), [&](vec2i p) {
        auto& sampler = samplers[threadIdx].start_pixel(p, current_sample_index);
        auto p_film = vec2(p + sampler.get2d()) / film.size();
        auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
        auto [L, _] = radiance(scene, ray, sampler, Vertex::first_vertex());
        film.add_sample(p, L);
      });
      current_sample_index += 1;
      current_samples += area(film.size());
      set_progress(current_samples / total_samples);
    }
  }

  set_progress(1.0f);
}

GuidedPathIntegrator::RadianceResult GuidedPathIntegrator::radiance(Scene& scene, Ray ray,
                                                                    Sampler& sampler, Vertex pv) {
  auto Lo = vec3(0.0f);
  auto wi = -ray.d;
  auto it = Interaction();

  if (!intersect(ray, it)) {
    if (scene.env_light)
      Lo += scene.env_light->color(wi);
    return {Lo, psl::nullopt};
  }

  if (it.material()->is<EmissiveMaterial>()) {
    Lo += it.material()->le({it, wi});
    if (pv.is_delta)
      return {Lo, psl::nullopt};
    auto light_pdf = it.geometry->pdf(it, ray, pv.n);
    auto mis_term = balance_heuristic(pv.pdf, light_pdf);
    return {Lo, mis_term};
  }

  if (pv.length + 1 >= max_path_length)
    return {Lo, psl::nullopt};

  auto& leaf = guide.traverse(it.p);

  if (!it.material()->is_delta())
    if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
      if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
        auto cosine = absdot(ls->wo, it.n);
        auto [f, bsdf_pdf] = it.material()->f_pdf({it, wi, ls->wo});
        auto mis = balance_heuristic(ls->pdf, bsdf_pdf);
        if (collect_radiance_sample)
          guide.add_sample(leaf, it.p, RadianceSample(ls->wo, ls->le / ls->pdf), sampler.get3d());
        Lo += ls->le * cosine * f / ls->pdf * mis;
      }
    }

  auto prob_a = it.material()->is_delta() ? 0.0f : use_learned_ratio;
  if (prob_a > 0.0f && sampler.get1d() < prob_a) {
    if (auto gs = leaf.sample(sampler.get2d())) {
      auto [Li, mis_direct] = radiance(scene, it.spawn_ray(gs->wo), sampler,
                                       Vertex(pv.length + 1, it.n, it.p, gs->pdf));
      auto cosine = absdot(gs->wo, it.n);
      auto mec = MaterialEvalCtx(it, wi, gs->wo);
      auto f = it.material()->f(mec);
      if (collect_radiance_sample)
        guide.add_sample(leaf, it.p, RadianceSample(gs->wo, Li / gs->pdf), sampler.get3d());
      if (mis_direct) {
        Lo += Li * cosine * f / gs->pdf * (*mis_direct);
      } else {
        auto bsdf_pdf = it.material()->pdf(mec);
        auto mis_indirect = prob_a == 1.0f ? 1.0f : balance_heuristic(gs->pdf, bsdf_pdf);
        Lo += Li * cosine * f / gs->pdf * mis_indirect / prob_a;
      }
    }
  } else {
    if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
      auto nv = Vertex(pv.length + 1, it.n, it.p, bs->pdf, it.material()->is_delta());
      auto [Li, mis_direct] = radiance(scene, it.spawn_ray(bs->wo), sampler, nv);
      auto cosine = absdot(bs->wo, it.n);
      if (collect_radiance_sample && !it.material()->is_delta())
        guide.add_sample(leaf, it.p, RadianceSample(bs->wo, Li / bs->pdf), sampler.get3d());
      if (mis_direct) {
        Lo += Li * cosine * bs->f / bs->pdf * (*mis_direct);
      } else {
        auto guide_pdf = leaf.pdf(bs->wo);
        auto mis_indirect = prob_a == 0.0f ? 1.0f : balance_heuristic(bs->pdf, guide_pdf);
        Lo += Li * cosine * bs->f / bs->pdf * mis_indirect / (1 - prob_a);
      }
    }
  }

  return {Lo, psl::nullopt};
}

}  // namespace pine