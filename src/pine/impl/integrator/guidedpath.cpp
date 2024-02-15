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
  for (int iteration = 0; iteration < n_iterations; iteration++) {
    auto iter_samples = initial_samples * (1 << iteration);

    {
      Profiler _("Collecting");
      auto downscale = psl::min(psl::sqrt(float(iter_samples) / total_pixels), 1.0f);
      auto iter_size = vec2i(film.size() * downscale);
      auto iter_n_pass = iter_samples / area(iter_size);
      for (size_t si = 0; si < iter_n_pass; si++) {
        parallel_for(iter_size, [&](vec2i p) {
          auto& sampler = samplers[threadIdx].start_pixel(p, current_sample_index);
          auto p_film = vec2(p + sampler.get2d()) / iter_size;
          auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
          auto L = radiance(scene, ray, sampler, Vertex{.length = 0});
          if (!use_estimate)
            film.add_sample(p_film * film.size(), L);
        });
        current_sample_index += 1;
        current_samples += area(iter_size);
        set_progress(current_samples / total_samples);
      }
    }
    {
      Profiler _("Refinement");
      guide.refine(iteration);
    }
  }

  if (use_estimate) {
    for (int si = 0; si < estimate_samples; si++) {
      parallel_for(film.size(), [&](vec2i p) {
        auto& sampler = samplers[threadIdx].start_pixel(p, current_sample_index);
        auto p_film = vec2(p + sampler.get2d()) / film.size();
        auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
        auto L = radiance_estimate(scene, ray, sampler, 0);
        film.add_sample(p, L);
      });
      current_sample_index += 1;
      current_samples += area(film.size());
      set_progress(current_samples / total_samples);
    }
  }

  set_progress(1.0f);
}

vec3 GuidedPathIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler, Vertex vertex) {
  auto wi = -ray.d;
  auto it = Interaction();
  if (!intersect(ray, it)) {
    if (scene.env_light)
      return scene.env_light->color(wi);
    else
      return vec3(0);
  }

  if (it.material()->is<EmissiveMaterial>())
    return it.material()->le({it, wi});

  if (vertex.length + 1 >= max_depth)
    return vec3(0);

  // if (sampler.get1d() < 0.5f) {
  if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
    auto li = radiance(scene, it.spawn_ray(bs->wo), sampler, Vertex{.length = vertex.length + 1});
    auto cosine = absdot(bs->wo, it.n);
    guide.add_sample(RadianceSample(it.p, bs->wo, li / bs->pdf), sampler.get3d());
    return li * cosine * bs->f / bs->pdf;
  } else {
    return vec3(0);
  }
  // }

  if (auto qs = guide.sample(it.p, sampler.get2d())) {
    auto li = radiance(scene, it.spawn_ray(qs->wo), sampler, Vertex{.length = vertex.length + 1});
    auto cosine = absdot(qs->wo, it.n);
    auto f = it.material()->f({it, wi, qs->wo});
    guide.add_sample(RadianceSample(it.p, qs->wo, li / qs->pdf), sampler.get3d());
    return li * cosine * f / qs->pdf;
  } else {
    return vec3(0);
  }
}

vec3 GuidedPathIntegrator::radiance_estimate(Scene& scene, Ray ray, Sampler& sampler, int depth) {
  auto wi = -ray.d;
  auto L = vec3{0.0f};

  auto it = Interaction();
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
    return vec3(0);

  it.n = face_same_hemisphere(it.n, wi);

  if (it.material()->is_delta()) {
    if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
      auto li = radiance_estimate(scene, it.spawn_ray(bs->wo), sampler, depth + 1);
      auto cosine = absdot(it.n, bs->wo);
      return li * cosine * bs->f / bs->pdf;
    } else {
      return vec3(0);
    }
  }

  // return guide.flux_estimate(it.p) * it.material()->f({it, it.n, it.n});

  if (auto qs = guide.sample(it.p, sampler.get2d())) {
    auto li = radiance_estimate(scene, it.spawn_ray(qs->wo), sampler, depth + 1);
    auto cosine = absdot(it.n, qs->wo);
    auto f = it.material()->f({it, wi, qs->wo});
    L += li * cosine * f / qs->pdf;
  }

  return L;
}

}  // namespace pine