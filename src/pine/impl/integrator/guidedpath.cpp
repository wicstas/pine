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

  Profiler _("[Integrator]Rendering");

  auto total_pixels = size_t(area(film.size()));
  auto total_samples = double(total_pixels * samples_per_pixel);
  auto initial_samples = size_t(1024 * 16);

  guide = SpatialTree(scene.get_aabb(), QuadTree());
  guide.initial_refinement(initial_samples * 4);

  auto current_sample_index = 0;
  auto current_samples = size_t(0);
  auto weights = psl::vector<float>();
  auto images = psl::vector<Array2d3f>();
  auto image = Array2d3f(film.size());
  auto squared_image = Array2d3f(film.size());
  auto prev_variance = 1e+10;
  auto on_final_rendering = false;
  auto stablizer = 0;

  set_progress(0);
  collect_radiance_sample = true;
  use_learned_ratio = 0.0f;

  for (int iteration = 0; current_samples < total_samples; iteration++) {
    Debug("iter: ", iteration, ", max_tree_depth: ", guide.max_tree_depth(),
          ", max_sample_count: ", guide.max_sample_count(), ", node_count: ", guide.node_count(),
          ", max_quad_node_count: ", guide.max_quad_node_count());
    auto iter_samples = initial_samples * (1 << iteration);
    auto downscale = psl::min(psl::sqrt(float(iter_samples) / total_pixels), 1.0f);
    auto iter_image_size = vec2i(film.size() * downscale);
    auto iter_n_pass = iter_samples / area(iter_image_size);
    iter_samples = iter_n_pass * area(iter_image_size);

    use_learned_ratio = iter_n_pass > 1 ? 0.5f : 0.0f;
    if (current_samples + iter_samples * 2.0 >= total_samples)
      on_final_rendering = true;
    if (on_final_rendering) {
      iter_image_size = film.size();
      iter_samples = iter_n_pass * area(iter_image_size);
      collect_radiance_sample = false;
    }

    size_t si = 0;
    while (true) {
      parallel_for(iter_image_size, [&](vec2i p) {
        auto& sampler = samplers[threadIdx].start_pixel(p, current_sample_index);
        auto p_film = vec2(p + sampler.get2d()) / iter_image_size;
        auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
        auto [L, _] = radiance(scene, ray, sampler, Vertex::first_vertex());
        image[p] += L;
        squared_image[p] += L * L;
      });
      current_sample_index += 1;
      current_samples += area(iter_image_size);
      set_progress(current_samples / total_samples);
      si++;
      if (!on_final_rendering && si >= iter_n_pass)
        break;
      if (on_final_rendering && si >= iter_n_pass && current_samples >= total_samples)
        break;
    }
    if (si != 1) {
      image /= si;
      squared_image /= si;
    }

    if (iter_n_pass > 1 || on_final_rendering) {
      auto variance = 0.0;
      if (iter_n_pass > 1) {
        parallel_for(iter_image_size, [&](vec2i p) {
          auto s_c = squared_image[p];
          auto c = image[p];
          auto local_variance = psl::sqr(luminance(abs(s_c - c * c)));
          variance += double(local_variance);
        });
        variance = variance / (area(iter_image_size) * psl::max(si - 1, size_t(1)));
      } else {
        variance = 1.0f;
      }
      weights.push_back(1.0f / variance);
      images.push_back(image);

      if (!on_final_rendering) {
        Debug("Variance: ", float(variance));
        if (variance > prev_variance / 2.0f) {
          stablizer++;
          if (stablizer > 1) {
            on_final_rendering = true;
            Debug("Done learning, start rendering");
          }
          prev_variance = psl::min(variance, prev_variance);
        } else {
          stablizer = 0;
          prev_variance = variance;
        }
      }
    }
    if (!on_final_rendering)
      guide.refine(iteration);
    for (auto& pixel : image)
      pixel = vec3(0);
    for (auto& pixel : squared_image)
      pixel = vec3(0);
  }

  auto total_weight = psl::sum(weights);
  for (auto& w : weights)
    w /= total_weight;
  Debug(weights);
  for_2d(film.size(), [&](vec2i p) {
    for (size_t i = 0; i < images.size(); i++)
      film.add_sample_no_acc(p, images[i][p] * weights[i]);
  });

  set_progress(1.0f);
}

GuidedPathIntegrator::RadianceResult GuidedPathIntegrator::radiance(Scene& scene, Ray ray,
                                                                    Sampler& sampler, Vertex pv) {
  auto Lo = vec3(0.0f);
  auto wi = -ray.d;
  auto it = Interaction();

  if (!intersect(ray, it)) {
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

  // Add irradiance in one call?
  if (select_guide) {
    if (auto gs = leaf.sample(sampler.get2d())) {
      auto nv = Vertex(pv.length + 1, it.n, it.p, gs->pdf);
      auto [Li, mis_direct] = radiance(scene, it.spawn_ray(gs->wo), sampler, nv);
      auto cosine = absdot(gs->wo, it.n);
      auto mec = MaterialEvalCtx(it, wi, gs->wo);
      auto f = it.material()->f(mec);
      if (mis_direct) {
      } else {
        auto bsdf_pdf = it.material()->pdf(mec);
        auto mis_indirect = prob_a == 1.0f ? 1.0f : balance_heuristic(gs->pdf, bsdf_pdf);
        Lo += Li * cosine * f / gs->pdf * mis_indirect / prob_a;
        if (collect_radiance_sample)
          guide.add_sample(leaf, it.p, RadianceSample(gs->wo, Li / gs->pdf * mis_indirect / prob_a),
                           sampler.get3d(), sampler.get2d());
      }
    }
  } else {
    if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
      auto nv = Vertex(pv.length + 1, it.n, it.p, bs->pdf, it.material()->is_delta());
      auto [Li, mis_direct] = radiance(scene, it.spawn_ray(bs->wo), sampler, nv);
      auto cosine = absdot(bs->wo, it.n);
      if (mis_direct) {
        Lo += Li * cosine * bs->f / bs->pdf * (*mis_direct);
      } else {
        auto guide_pdf = leaf.pdf(bs->wo);
        auto mis_indirect = prob_a == 0.0f ? 1.0f : balance_heuristic(bs->pdf, guide_pdf);
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