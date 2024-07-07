#include <pine/core/integrator.h>
#include <pine/core/sampling.h>
#include <pine/core/parallel.h>
#include <pine/core/profiler.h>
#include <pine/core/sampler.h>
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

RTIntegrator::RTIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler)
    : accel{MOVE(accel)}, light_sampler{MOVE(light_sampler)} {
  spp = sampler.spp();
  samplers.push_back(MOVE(sampler));
}
void RTIntegrator::render(Scene& scene) {
  this->scene = &scene;
  auto sampler = samplers.consume_back();
  sampler.init(scene.camera.film().size());
  for (int i = 0; i < n_threads(); i++)
    samplers.push_back(sampler);
  accel.build(&scene);
  light_sampler.build(&scene);
  set_progress(0);
}
bool RTIntegrator::intersect(Ray& ray, SurfaceInteraction& it) const {
  auto is_hit = accel.intersect(ray, it);
  if (is_hit)
    it.compute_transformation();
  return is_hit;
}
psl::optional<SurfaceInteraction> RTIntegrator::intersect(Ray& ray) const {
  auto it = SurfaceInteraction();
  if (intersect(ray, it))
    return MOVE(it);
  else
    return psl::nullopt;
}
psl::optional<MediumSample> RTIntegrator::sample_medium(Ray ray, Sampler& sampler) const {
  auto N = scene->mediums.size();
  if (N) {
    auto t = -psl::log(1 - sampler.get1d());
    if (auto ls = light_sampler.sample(ray(t), sampler)) {
      auto ps = ray(t) + ls->distance * ls->wo;
      auto a = dot(ps - ray.o, ray.d);
      auto b = ray.tmax - a;
      auto D = length(ps - ray(a));
      auto theta_a = -psl::atan(a / D);
      auto theta_b = psl::atan(b / D);
      auto dt = D * psl::tan(lerp(sampler.get1d(), theta_b, theta_a));
      auto ts = a + dt;

      auto pdf = D / ((theta_b - theta_a) * psl::sqrt(D * D + dt * dt));

      auto index = int(N * sampler.get1d());
      return MediumSample(
          ray(ts),
          scene->mediums[index].at(ray(ts)).sigma_s * transmittance(ray.o, ray.d, ts, sampler),
          pdf / N, scene->mediums[index].get_pf());
    }
  }

  return psl::nullopt;
}
vec3 RTIntegrator::transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler, int) const {
  auto tr = vec3(1.0f);
  auto N = int(scene->mediums.size());
  for (int i = 0; i < N; i++)
    tr *= scene->mediums[i].transmittance(p, d, tmax, sampler);
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
    for (int si = 0; si < spp; si++, sampler.start_next_sample()) {
      auto ray = scene.camera.gen_ray((p + sampler.rand2f()) / film.size(), sampler.rand2f());
      L += radiance(scene, ray, sampler);
    }
    scene.camera.film()[p] = vec4(L / spp, 1.0f);
    if (p.x % 64 == 0)
      set_progress(float(p.x + p.y * film.size().x) / area(film.size()));
  });
}

}  // namespace pine
