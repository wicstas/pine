#include <pine/impl/integrator/spatial_tree.h>
#include <pine/impl/integrator/guidedpath.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

namespace pine {

static SpatialTree sd_tree;

static bool use_guide = false;
static bool collect = true;

void GuidedPathIntegrator::render(Scene& scene) {
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("GuidedPathIntegrator doesn't support `Plane`, please use `Rect` or `Disk` instead");

  accel.build(&scene);
  light_sampler.build(&scene);
  auto& film = scene.camera.film();
  film.clear();

  Profiler _("Rendering");

  auto quad_tree = QuadTree();
  quad_tree.root().flux = 1;
  quad_tree.refine();
  sd_tree = SpatialTree{scene.get_aabb(), quad_tree};
  use_guide = false;
  collect = true;

  auto total_pixels = static_cast<size_t>(area(film.size()));
  auto total_samples = total_pixels * samples_per_pixel;
  auto total_samples_all = total_samples + total_pixels * estimate_samples;
  auto initial_samples = size_t{1024 * 16};
  auto n_iterations =
      static_cast<int>(psl::ceil(psl::log2(total_samples / initial_samples + 1.0f)));
  Debug("[GuidedPath]", n_iterations, " learning iterations");
  sd_tree.root().n_samples = initial_samples * 4;
  sd_tree.refine(0);

  auto current_sample_index = 0;
  auto current_samples = size_t{0};
  auto current_film = film;

  set_progress(0);
  for (int iteration = 0; iteration < n_iterations; iteration++) {
    auto iter_samples = initial_samples * (1 << iteration);

    if (iteration + 1 == n_iterations) {
      if (!use_estimate)
        collect = false;
    }

    {
      Profiler _("Collecting");
      auto downsize = psl::min(psl::sqrt(static_cast<double>(iter_samples) / total_pixels), 1.0);
      auto iter_size = vec2i{film.size() * downsize};
      auto iter_n_pass = iter_samples / area(iter_size);
      for (size_t si = 0; si < iter_n_pass; si++) {
        parallel_for(iter_size, [&](vec2i p) {
          auto& sampler = samplers[threadIdx];
          sampler.start_pixel(p, current_sample_index);

          auto p_film = vec2(p + sampler.get2d()) / iter_size;
          auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
          auto L = radiance(scene, ray, sampler, 0, 1.0f, vec3{0.0f}, true, false);
          if (!use_estimate)
            current_film.add_sample(p_film * current_film.size(), L);
        });
        current_sample_index += 1;
        current_samples += area(iter_size);
        set_progress(static_cast<double>(current_samples) / total_samples_all);
      }

      if (!use_estimate) {
        if (iteration == 0) {
          film = current_film;
        } else {
          film = combine(film, current_film, 1, 2);
        }
        current_film.clear();
      }
    }
    {
      Profiler _("Refinement");
      if (collect)
        sd_tree.refine(iteration);
      use_guide = true;
    }
  }

  if (use_estimate) {
    film.clear();
    for (int si = 0; si < estimate_samples; si++) {
      parallel_for(film.size(), [&](vec2i p) {
        auto& sampler = samplers[threadIdx];
        sampler.start_pixel(p, current_sample_index);
        auto p_film = vec2(p + sampler.get2d()) / film.size();
        auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
        auto L = radiance_estimate(scene, ray, sampler, 0);
        L = clamp(L, vec3{0.0f}, vec3{20.0f});
        film.add_sample(p, L);
      });
      current_sample_index += 1;
      current_samples += area(film.size());
      set_progress(static_cast<double>(current_samples) / total_samples_all);
    }
  }

  set_progress(1.0f);
}

vec3 GuidedPathIntegrator::radiance_estimate(Scene& scene, Ray ray, Sampler& sampler, int depth) {
  auto L = vec3{0.0f};

  auto it = Interaction{};
  if (!intersect(ray, it)) {
    if (scene.env_light)
      L += scene.env_light->color(ray.d);
    return L;
  }

  if (it.material()->is<EmissiveMaterial>()) {
    L += it.material()->le({it, -ray.d});
    return L;
  }

  if (depth + 1 == max_depth)
    return vec3{0.0f};

  if (dot(it.n, -ray.d) < 0)
    it.n = -it.n;

  if (it.material()->is_delta()) {
    auto msc = MaterialSampleCtx{it, -ray.d, sampler.get1d(), sampler.get2d()};
    if (auto bs = it.material()->sample(msc)) {
      auto li = radiance_estimate(scene, it.spawn_ray(bs->wo), sampler, depth + 1);
      auto cosine = absdot(it.n, bs->wo);
      return li * cosine * bs->f / bs->pdf;
    } else {
      return vec3{0.0f};
    }
  }

  auto direct_light = [&](LightSample ls) {
    if (!hit(it.spawn_ray(ls.wo, ls.distance))) {
      auto mec = MaterialEvalCtx(it, -ray.d, ls.wo);
      auto f = it.material()->f(mec);
      return ls.le / ls.pdf * psl::max(dot(it.n, ls.wo), 0.0f) * f;
    } else {
      return vec3{0.0f};
    }
  };

  auto footprint = sd_tree.traverse(it.p).get_footprint();
  auto p = it.p + footprint * (sampler.get3d() - vec3{0.5f}) * 2;
  auto aabb = sd_tree.root().aabb;
  for (int i = 0; i < 3; i++) {
    if (p[i] < aabb.lower[i])
      p[i] += 2 * (aabb.lower[i] - p[i]);
    else if (p[i] > aabb.upper[i])
      p[i] += 2 * (aabb.upper[i] - p[i]);
  }
  auto& quad = *sd_tree.traverse(p).guide;

  auto guide_select_prob = 0.8f;
  if (sampler.get1d() < guide_select_prob) {
    if (auto qs = quad.sample(sampler.get2d())) {
      auto wo = uniform_sphere(qs->sc);
      auto mec = MaterialEvalCtx{it, -ray.d, wo};
      auto le = quad.flux_estimate(qs->sc) / (4);
      auto mis_term = balance_heuristic(1, qs->pdf, 1, it.material()->pdf(mec)) / guide_select_prob;
      L += le * it.material()->f(mec) / qs->pdf * mis_term;
    }
  } else {
    if (auto bs = it.material()->sample({it, -ray.d, sampler.get1d(), sampler.get2d()})) {
      auto sc = inverse_uniform_sphere(bs->wo);
      auto le = quad.flux_estimate(sc) / (4);
      auto mis_term = balance_heuristic(1, bs->pdf, 1, quad.pdf(sc)) / (1 - guide_select_prob);
      L += le * bs->f / bs->pdf * mis_term;
    }
  }
  if (scene.env_light)
    if (auto ls = scene.env_light->sample(it.n, sampler.get2d()))
      L += direct_light(*ls);

  return L;
}

vec3 GuidedPathIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler, int depth,
                                    float prev_sample_pdf, vec3 prev_n, bool prev_delta,
                                    bool mis_env_light) {
  auto wi = -ray.d;
  auto it = Interaction{};
  if (!intersect(ray, it)) {
    if (scene.env_light) {
      if (prev_delta) {
        return scene.env_light->color(ray.d);
      } else if (mis_env_light) {
        auto le = scene.env_light->color(ray.d);
        auto light_pdf = scene.env_light->pdf(prev_n, ray.d);
        auto mis_term = power_heuristic(1, prev_sample_pdf, 1, light_pdf);
        // mis_term shouldn't be added to sd_tree?
        return le * mis_term;
      }
    }
    return vec3{0.0f};
  }

  if (it.material()->is<EmissiveMaterial>()) {
    if (prev_delta) {
      return it.material()->le({it, -wi});
    } else if (mis_env_light) {
      auto le = it.material()->le({it, -wi});
      auto light_pdf = light_sampler.pdf(it.geometry, it, ray, prev_n);
      auto mis_term = power_heuristic(1, prev_sample_pdf, 1, light_pdf);
      return le * mis_term;
    }
    return vec3{0.0f};
  }

  if (depth + 1 == max_depth)
    return vec3{0.0f};

  auto lo = vec3{0.0f};

  SpatialNode& leaf = sd_tree.traverse(it.p);
  auto direct_light = [&](LightSample ls, float pdf_g, bool collect_ = true) {
    if (!hit(it.spawn_ray(ls.wo, ls.distance))) {
      auto cosine = absdot(ls.wo, it.n);
      auto mec = MaterialEvalCtx(it, -ray.d, ls.wo);
      auto f = it.material()->f(mec);
      auto mis_term = power_heuristic(ls.pdf, pdf_g);
      // TODO: divides by pdf?
      if (collect && collect_)
        sd_tree.add_sample(leaf, {it.p, ls.wo, ls.le * cosine / ls.pdf}, sampler.get3d());
      return ls.le / ls.pdf * cosine * f * mis_term;
    } else {
      return vec3{0.0f};
    }
  };

  auto guide_select_prob = use_guide ? 0.75f : 0.0f;
  if (it.material()->is_delta())
    guide_select_prob = 0.0f;
  if (sampler.get1d() < guide_select_prob) {
    if (auto ps = leaf.sample(sampler.get2d())) {
      auto cosine = absdot(it.n, ps->w);
      auto mec = MaterialEvalCtx{it, wi, ps->w};
      auto f = it.material()->f(mec);
      if (psl::abs(ps->pdf) > 1e-7f && cosine > 1e-7f && length_squared(f) > 1e-10f) {
        auto li =
            radiance(scene, it.spawn_ray(ps->w), sampler, depth + 1, 0.0f, it.n, false, false);
        if (collect)
          sd_tree.add_sample(leaf, {it.p, ps->w, li * cosine / ps->pdf}, sampler.get3d());
        auto mis_term = power_heuristic(1, ps->pdf, 1, it.material()->pdf(mec)) / guide_select_prob;
        lo += li * cosine * f / ps->pdf * mis_term;
      }
    }

    if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
      lo += direct_light(*ls, 0.0f, false);
    }
    if (scene.env_light) {
      if (auto ls = scene.env_light->sample(it.n, sampler.get2d()))
        lo += direct_light(*ls, 0.0f, false);
    }
  } else {
    if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
      auto cosine = absdot(it.n, bs->wo);
      if (psl::abs(bs->pdf) > 1e-7f && cosine > 1e-7f && length_squared(bs->f) > 1e-10f) {
        auto li = radiance(scene, it.spawn_ray(bs->wo), sampler, depth + 1, bs->pdf, it.n,
                           it.material()->is_delta(), true);
        if (collect)
          sd_tree.add_sample(leaf, {it.p, bs->wo, li * cosine / bs->pdf}, sampler.get3d());
        auto mis_term =
            guide_select_prob == 0.0f
                ? 1.0f
                : power_heuristic(1, bs->pdf, 1, leaf.pdf(bs->wo)) / (1 - guide_select_prob);
        lo += li * cosine * bs->f / bs->pdf * mis_term;
      }
    }

    if (!it.material()->is_delta())
      if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
        lo += direct_light(*ls, ls->light->is_delta() ? 0.0f : it.material()->pdf({it, wi, ls->wo}),
                           false);
      }
    if (!it.material()->is_delta() && scene.env_light) {
      if (auto ls = scene.env_light->sample(it.n, sampler.get2d()))
        lo += direct_light(*ls, it.material()->pdf({it, wi, ls->wo}), false);
    }
  }

  return lo;
}

}  // namespace pine