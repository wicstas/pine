#include <pine/impl/integrator/path.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>

#include <cfenv>

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
  Vertex(int length, Interaction it, float pdf, bool is_delta = false,
         bool inside_subsurface = false)
      : length(length),
        it(psl::move(it)),
        pdf(pdf),
        is_delta(is_delta),
        inside_subsurface(inside_subsurface) {
  }
  static Vertex first_vertex() {
    return Vertex(0, {}, 0.0f, true);
  }
  int length;
  Interaction it;
  float pdf;
  bool is_delta;
  bool inside_subsurface;
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

  auto [mit, it] = intersect_tr(ray, sampler);

  if (mit) {
    auto Tr = transmittance(ray.o, ray.d, mit->t, sampler, mit->medium_index);
    if (mit->le)
      Lo += *mit->le * Tr * mit->W;
    else if (pv.length + 1 < max_path_length) {
      if (!pv.inside_subsurface)
        if (auto ls = light_sampler.sample(*mit, sampler.get1d(), sampler.get2d())) {
          if (!hit(Ray(mit->p, ls->wo, 0.0f, ls->distance))) {
            auto tr = transmittance(mit->p, ls->wo, ls->distance, sampler);
            auto f = mit->pg.f(wi, ls->wo);
            if (ls->light->is_delta()) {
              Lo += ls->le * Tr * mit->W * tr * f / ls->pdf;
            } else {
              auto mis = balance_heuristic(ls->pdf, mit->pg.pdf(wi, ls->wo));
              Lo += ls->le * Tr * mit->W * tr * f / ls->pdf * mis;
            }
          }
        }
      auto ps = mit->pg.sample(wi, sampler.get2d());
      auto nv = Vertex(pv.length + 1, *mit, ps.pdf, pv.inside_subsurface);
      auto rr = pv.length <= 1 ? 1.0f : psl::max(ps.f / ps.pdf, 0.05f);
      if (rr >= 1 || sampler.get1d() < rr) {
        auto [Li, light_pdf] = radiance(scene, Ray(mit->p, ps.wo), sampler, nv);
        auto mis = light_pdf ? balance_heuristic(ps.pdf, *light_pdf) : 1.0f;
        Lo += Li * Tr * mit->W * (ps.f / ps.pdf * mis / psl::min(1.0f, rr));
      }
    }
  }

  auto Tr = transmittance(ray.o, ray.d, ray.tmax, sampler);
  if (!it) {
    if (scene.env_light) {
      Lo += Tr * scene.env_light->color(ray.d);
      if (!pv.is_delta)
        result.light_pdf = scene.env_light->pdf(pv.it, ray.d);
    }
    return result;
  }

  if (it->material().is<EmissiveMaterial>()) {
    Lo += Tr * it->material().le({*it, wi});
    if (!pv.is_delta)
      result.light_pdf = light_sampler.pdf(pv.it, *it, ray);
    return result;
  }

  if (pv.length + 1 >= max_path_length)
    return result;

  auto rr_m = average(Tr);
  auto lo = vec3(0.0f);
  if (with_probability(rr_m, sampler)) {
    if (!it->material().is_delta(*it)) {
      if (auto ls = light_sampler.sample(*it, sampler.get1d(), sampler.get2d())) {
        if (!hit(it->spawn_ray(ls->wo, ls->distance))) {
          auto cosine = absdot(ls->wo, it->n);
          auto tr = transmittance(it->p, ls->wo, ls->distance, sampler);
          if (ls->light->is_delta()) {
            auto f = it->material().f({*it, wi, ls->wo, pv.is_delta ? 0.0f : 0.2f});
            lo += Tr * ls->le * tr * cosine * f / ls->pdf;
          } else {
            auto [f, bsdf_pdf] = it->material().f_pdf({*it, wi, ls->wo, pv.is_delta ? 0.0f : 0.2f});
            auto mis = balance_heuristic(ls->pdf, bsdf_pdf);
            lo += Tr * ls->le * tr * cosine * f / ls->pdf * mis;
          }
        }
      }
    }

    if (auto bs = it->material().sample({*it, wi, sampler.get1d(), sampler.get2d()})) {
      auto cosine = absdot(bs->wo, it->n);
      auto nv =
          Vertex(pv.length + 1, *it, bs->pdf, it->material().is_delta(*it), bs->enter_subsurface);
      auto rr = pv.length <= 1 ? 1.0f : psl::max(luminance(cosine * bs->f / bs->pdf), 0.05f);
      if (rr >= 1 || sampler.get1d() < rr) {
        auto [Li, light_pdf] = radiance(scene, it->spawn_ray(bs->wo), sampler, nv);
        auto mis = light_pdf ? balance_heuristic(bs->pdf, *light_pdf) : 1.0f;
        lo += Tr * Li * bs->f * (cosine / bs->pdf * mis / psl::min(1.0f, rr));
      }
    }
  }
  if (rr_m)
    Lo += lo / rr_m;

  return result;
}

}  // namespace pine