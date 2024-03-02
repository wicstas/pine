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
  spp = sampler.spp();
}

void RayIntegrator::render(Scene& scene) {
  accel.build(&scene);
  auto& film = scene.camera.film();
  film.clear();
  set_progress(0);

  Profiler _("[Integrator]Rendering");
  for (int si = 0; si < spp; si++) {
    Atomic<int64_t> max_index = 0;
    parallel_for(film.size(), [&](vec2i p) {
      Sampler& sampler = samplers[threadIdx];
      sampler.start_pixel(p, si);
      auto p_film = vec2(p + sampler.get2d()) / film.size();
      auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
      auto L = radiance(scene, ray, sampler);
      scene.camera.film().add_sample(p, L);
      if (p.x == 0) {
        max_index = psl::max<int64_t>(max_index, p.x + p.y * film.size().x);
        set_progress(float(si) / spp + float(max_index) / area(film.size()) / spp);
      }
    });
  }

  set_progress(1);
}

}  // namespace pine
