#include <pine/core/integrator.h>
#include <pine/core/sampling.h>
#include <pine/core/parallel.h>
#include <pine/core/profiler.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

namespace pine {

Atomic<float> g_progress{};

void set_progress(float progress) {
  if (progress == 0 || progress > g_progress)
    g_progress = psl::clamp(progress, 0.0f, 1.0f);
}
float get_progress() {
  return g_progress;
}

RTIntegrator::RTIntegrator(Accel accel, Sampler sampler) : accel{psl::move(accel)} {
  spp = sampler.spp();
  samplers.push_back(psl::move(sampler));
}
void RTIntegrator::render(Scene& scene) {
  this->scene = &scene;
  auto sampler = samplers.consume_back();
  sampler.init(scene.camera.film().size());
  for (int i = 0; i < n_threads(); i++)
    samplers.push_back(sampler);
  accel.build(&scene);
  set_progress(0);
}
bool RTIntegrator::intersect(Ray& ray, SurfaceInteraction& it) const {
  auto is_hit = accel.intersect(ray, it);
  if (is_hit)
    it.compute_transformation();
  return is_hit;
}
psl::optional<Interaction> RTIntegrator::intersect_tr(Ray& ray, Sampler& sampler) const {
  auto ms = psl::optional<MediumInteraction>();
  for (auto& medium : scene->mediums) {
    if (auto ms_ = medium.intersect_tr(ray, sampler))
      ms = *psl::move(ms_);
  }
  auto it = SurfaceInteraction();
  if (intersect(ray, it))
    return it;
  else if (ms)
    return *ms;
  else
    return psl::nullopt;
}
vec3 RTIntegrator::transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const {
  auto tr = vec3(1.0f);
  for (const auto& medium : scene->mediums)
    tr *= medium.transmittance(p, d, tmax, sampler);
  return tr;
}

void RayIntegrator::render(Scene& scene) {
  RTIntegrator::render(scene);
  auto& film = scene.camera.film();
  film.clear();

  Profiler _("[Ray]Render");
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = samplers[threadIdx].start_pixel(p, 0);
    auto L = vec3(0.0f);
    for (int si = 0; si < spp; si++) {
      auto ray = scene.camera.gen_ray((p + sampler.get2d()) / film.size(), sampler.get2d());
      L += radiance(scene, ray, sampler);
      sampler.start_next_sample();
    }
    scene.camera.film()[p] = vec4(L / spp, 1.0f);
    if (p.x % 64 == 0)
      set_progress(float(p.x + p.y * film.size().x) / area(film.size()));
  });
}

}  // namespace pine
