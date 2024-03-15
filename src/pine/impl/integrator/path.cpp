#include <pine/impl/integrator/path.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>

namespace pine {

PathIntegrator::PathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                               int max_path_length)
    : RTIntegrator{psl::move(accel), psl::move(sampler)},
      light_sampler{psl::move(light_sampler)},
      max_path_length{max_path_length} {
  if (max_path_length <= 0)
    Fatal("`PathIntegrator` expect `max_path_length` to be positive, get", max_path_length);
}
struct PathIntegrator::Vertex {
  Vertex(int length, vec3 n, vec3 p, float pdf, bool is_delta = false)
      : length(length), n(n), p(p), pdf(pdf), is_delta(is_delta) {
  }
  static Vertex first_vertex() {
    return Vertex(0, vec3(0), vec3(0), 0.0f, true);
  }
  int length;
  vec3 n;
  vec3 p;
  float pdf;
  bool is_delta;
};
void PathIntegrator::render(Scene& scene) {
  RTIntegrator::render(scene);
  light_sampler.build(&scene);
  auto& film = scene.camera.film();

  Profiler _("[Path]Render");
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = samplers[threadIdx].start_pixel(p, 0);
    auto L = vec3(0.0f);
    for (int si = 0; si < spp; si++, sampler.start_next_sample()) {
      auto ray = scene.camera.gen_ray((p + sampler.get2d()) / film.size(), sampler.get2d());
      L += radiance(scene, ray, sampler, Vertex::first_vertex()).Lo;
    }
    scene.camera.film()[p] = vec4(L / spp, 1.0f);
    if (p.x % 64 == 0)
      set_progress(float(p.x + p.y * film.size().x) / area(film.size()));
  });
}
PathIntegrator::RadianceResult PathIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler,
                                                        Vertex pv) {
  auto result = RadianceResult();
  auto wi = -ray.d;
  auto& Lo = result.Lo;

  auto tr0 = transmittance(ray.o, ray.d, ray.tmax, sampler);
  auto discontinue_path = false;

  auto it = SurfaceInteraction();
  if (!intersect(ray, it)) {
    if (scene.env_light) {
      Lo += tr0 * scene.env_light->color(ray.d);
      if (!pv.is_delta)
        result.light_pdf = scene.env_light->pdf(pv.n, ray.d);
    }
    discontinue_path = true;
  } else if (it.material()->is<EmissiveMaterial>()) {
    Lo += tr0 * it.material()->le({it, wi});
    if (!pv.is_delta)
      result.light_pdf = light_sampler.pdf(it.geometry, it, ray, pv.n);
    discontinue_path = true;
  }

  if (pv.length + 1 < max_path_length)
    if (auto ms = sample_medium(ray.o, ray.d, ray.tmax, sampler)) {
      if (auto ls = light_sampler.sample(ms->p, vec3(0.0f), sampler.get1d(), sampler.get2d())) {
        if (!hit(Ray(ms->p, ls->wo, 0.0f, ls->distance))) {
          auto tr = transmittance(ms->p, ls->wo, ls->distance, sampler);
          auto f = ms->pg.f(wi, ls->wo);
          if (ls->light->is_delta()) {
            Lo += ms->tr * ls->le * ms->sigma * tr * f / ls->pdf / ms->pdf;
          } else {
            auto mis = balance_heuristic(ls->pdf, ms->pg.pdf(wi, ls->wo));
            Lo += ms->tr * ls->le * ms->sigma * tr * f / ls->pdf / ms->pdf * mis;
          }
        }
      }
      auto ps = ms->pg.sample(wi, sampler.get2d());
      auto nv = Vertex(pv.length + 1, vec3(0.0f), ms->p, ps.pdf);
      auto [Li, light_pdf] = radiance(scene, Ray(ms->p, ps.wo), sampler, nv);
      auto mis = light_pdf ? balance_heuristic(ps.pdf, *light_pdf) : 1.0f;
      Lo += ms->tr * Li * ms->sigma * ps.f / ps.pdf * mis / ms->pdf;
    }

  if (discontinue_path || pv.length + 1 >= max_path_length)
    return result;

  if (!it.material()->is_delta()) {
    if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
      if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
        auto cosine = absdot(ls->wo, it.n);
        auto tr = transmittance(it.p, ls->wo, ls->distance, sampler);
        if (ls->light->is_delta()) {
          auto f = it.material()->f({it, wi, ls->wo});
          Lo += tr0 * ls->le * tr * cosine * f / ls->pdf;
        } else {
          auto [f, bsdf_pdf] = it.material()->f_pdf({it, wi, ls->wo});
          auto mis = balance_heuristic(ls->pdf, bsdf_pdf);
          Lo += tr0 * ls->le * tr * cosine * f / ls->pdf * mis;
        }
      }
    }
  }

  if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
    auto cosine = absdot(bs->wo, it.n);
    auto nv = Vertex(pv.length + 1, it.n, it.p, bs->pdf, it.material()->is_delta());
    auto [Li, light_pdf] = radiance(scene, it.spawn_ray(bs->wo), sampler, nv);
    auto mis = light_pdf ? balance_heuristic(bs->pdf, *light_pdf) : 1.0f;
    Lo += tr0 * Li * cosine * bs->f / bs->pdf * mis;
  }

  return result;
}

}  // namespace pine