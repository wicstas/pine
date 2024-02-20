#include <pine/core/integrator.h>
#include <pine/core/sampling.h>
#include <pine/core/parallel.h>
#include <pine/core/profiler.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

namespace pine {

Atomic<float> g_progress{};

void set_progress(float progress) {
  g_progress = psl::clamp(progress, 0.0f, 1.0f);
}
float get_progress() {
  return g_progress;
}

RTIntegrator::RTIntegrator(Accel accel, Sampler sampler) : accel{psl::move(accel)} {
  for (int i = 0; i < n_threads(); i++)
    samplers.push_back(sampler);
  samples_per_pixel = sampler.spp();
}

void RayIntegrator::render(Scene& scene) {
  accel.build(&scene);
  auto& film = scene.camera.film();
  film.clear();
  set_progress(0);

  auto primary_spp = psl::max(samples_per_pixel / primary_ratio, 1);
  auto secondary_spp = psl::max(samples_per_pixel / primary_spp, 1);
  Profiler _("[Integrator]Rendering");
  for (int i = 0; i < primary_spp; i++) {
    Atomic<int64_t> max_index = 0;
    parallel_for(film.size(), [&](vec2i p) {
      Sampler& sampler = samplers[threadIdx];
      sampler.start_pixel(p, i * primary_ratio);
      auto p_film = vec2(p + sampler.get2d()) / film.size();
      auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
      auto it = Interaction();
      auto is_hit = intersect(ray, it);
      for (int si = 0; si < secondary_spp; si++) {
        auto L = radiance(scene, ray, it, is_hit, sampler);
        scene.camera.film().add_sample(p, L);
        if (si != secondary_spp - 1)
          sampler.start_next_sample();
      }

      if (p.x == 0) {
        max_index = psl::max<int64_t>(max_index, p.x + p.y * film.size().x);
        set_progress(float(i) / primary_spp + float(max_index) / area(film.size()) / primary_spp);
      }
    });
    set_progress(static_cast<float>(i + 1) / primary_spp);
  }

  set_progress(1);
}

}  // namespace pine
