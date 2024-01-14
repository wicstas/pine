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
    samplers.push_back(sampler.Clone());
  samplesPerPixel = sampler.SamplesPerPixel();
}
bool RTIntegrator::hit(Ray ray) const {
  return accel.hit(ray);
}
bool RTIntegrator::intersect(Ray& ray, Interaction& it) const {
  return accel.intersect(ray, it);
}

void PixelIntegrator::render(Scene& scene) {
  accel.build(&scene);
  auto& film = scene.camera.film();
  film.clear();
  set_progress(0);

  Profiler _("Rendering");
  for (int i = 0; i < samplesPerPixel; i++) {
    ParallelFor(film.size(), [&](vec2i p) {
      Sampler& sampler = samplers[threadIdx];
      sampler.StartPixel(p, i);
      pixel_color(scene, p, sampler);
    });
    set_progress(static_cast<float>(i) / samplesPerPixel);
  }
}

void RayIntegrator::pixel_color(Scene& scene, vec2i p, Sampler& sampler) {
  auto p_film = vec2{p + sampler.Get2D()} / scene.camera.film().size();
  auto ray = scene.camera.gen_ray(p_film, sampler.Get2D());
  auto L = radiance(scene, ray, sampler);
  CHECK(!L.has_nan());
  CHECK(!L.has_inf());
  scene.camera.film().add_sample(p, L);
}

}  // namespace pine
