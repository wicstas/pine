#include <pine/impl/integrator/spatial_tree.h>
#include <pine/impl/integrator/guidedpath.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

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

    use_learned_ratio = psl::sqrt(psl::min(float(iteration) / n_iterations, 1.0f));
    if (is_last_iter) {
      if (!use_estimate)
        collect_radiance_sample = false;
      use_learned_ratio = 1.0f;
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
    use_learned_ratio = 1.0f;
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

  if (!it.material()->is_delta())
    if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
      if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
        auto cosine = absdot(ls->wo, it.n);
        auto [f, bsdf_pdf] = it.material()->f_pdf({it, wi, ls->wo});
        auto mis = balance_heuristic(ls->pdf, bsdf_pdf);
        if (collect_radiance_sample)
          guide.add_sample(RadianceSample(it.p, ls->wo, ls->le / ls->pdf), sampler.get3d());
        Lo += ls->le * cosine * f / ls->pdf * mis;
      }
    }

  if ((sampler.get1d() < use_learned_ratio) && !it.material()->is_delta()) {
    if (auto gs = guide.sample(it.p, sampler.get2d())) {
      auto [Li, mis] = radiance(scene, it.spawn_ray(gs->wo), sampler,
                                Vertex(pv.length + 1, it.n, it.p, gs->pdf));
      auto cosine = absdot(gs->wo, it.n);
      auto f = it.material()->f({it, wi, gs->wo});
      if (collect_radiance_sample)
        guide.add_sample(RadianceSample(it.p, gs->wo, Li / gs->pdf), sampler.get3d());
      Lo += Li * cosine * f / gs->pdf * (mis ? *mis : 1.0f);
    }
  } else {
    if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
      auto nv = Vertex(pv.length + 1, it.n, it.p, bs->pdf, it.material()->is_delta());
      auto [Li, mis] = radiance(scene, it.spawn_ray(bs->wo), sampler, nv);
      auto cosine = absdot(bs->wo, it.n);
      if (collect_radiance_sample && !it.material()->is_delta())
        guide.add_sample(RadianceSample(it.p, bs->wo, Li / bs->pdf), sampler.get3d());
      Lo += Li * cosine * bs->f / bs->pdf * (mis ? *mis : 1.0f);
    }
  }

  return {Lo, psl::nullopt};
}

}  // namespace pine