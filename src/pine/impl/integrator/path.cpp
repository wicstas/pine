#include <pine/impl/integrator/path.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>

namespace pine {

PathIntegrator::PathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                               int max_path_length)
    : RTIntegrator{MOVE(accel), MOVE(sampler), MOVE(light_sampler)},
      max_path_length{max_path_length} {
  if (max_path_length <= 0)
    Fatal("`PathIntegrator` expect `max_path_length` to be positive, get", max_path_length);
}
struct PathIntegrator::Vertex {
  Vertex(int length, int diffuse_length, float pdf, bool is_delta)
      : length(length), diffuse_length(diffuse_length), pdf(pdf), is_delta(is_delta) {
  }
  Vertex(const Vertex& pv, float pdf, bool is_delta = false)
      : Vertex(pv.length + 1, pv.diffuse_length + (is_delta ? 0 : 1), pdf, is_delta) {
  }
  static Vertex first_vertex() {
    return Vertex(0, 0, 0.0f, true);
  }
  int length;
  int diffuse_length;
  float pdf;
  bool is_delta;
};
void PathIntegrator::render(Scene& scene) {
  RTIntegrator::render(scene);
  auto& film = scene.camera.film();

  Profiler _("[Path]Render");
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = samplers[threadIdx].start_pixel(p, 0);
    auto L = vec3(0.0f);
    for (int si = 0; si < spp; si++, sampler.start_next_sample()) {
      auto ray = scene.camera.gen_ray((p + sampler.rand2f()) / film.size(), sampler.rand2f());
      L += radiance(scene, ray, sampler, Vertex::first_vertex()).Lo;
    }
    film[p] = vec4(L / spp, 1.0f);
    if (p.x % 64 == 0)
      set_progress(progress_2d(p, film.size()));
  });
}
PathIntegrator::RadianceResult PathIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler,
                                                        Vertex pv) {
  auto result = RadianceResult();
  auto wi = -ray.d;
  auto& Lo = result.Lo;

  auto it = intersect(ray);
  // if (it) {
  //   Lo = it->n / 2 + vec3(0.5f);
  //   return result;
  // }

  if (pv.diffuse_length == 0)
    if (auto mit = sample_medium(ray, sampler)) {
      if (pv.length + 1 < max_path_length) {
        auto lo = vec3(0.0f);
        if (auto ls = light_sampler.sample(mit->p, sampler);
            ls && !hit(mit->spawn_ray(ls->wo, ls->distance))) {
          auto tr = transmittance(mit->p, ls->wo, ls->distance, sampler);
          auto f = mit->pg.f(wi, ls->wo);
          if (ls->light->is_delta())
            lo += ls->le * tr * (f / ls->pdf);
          else
            lo += ls->le * tr * f / ls->pdf * balance_heuristic(ls->pdf, mit->pg.pdf(wi, ls->wo));
        }
        {
          auto ps = mit->pg.sample(wi, sampler.get2d());
          auto nv = Vertex(pv, ps.pdf, false);
          auto [Li, light_pdf] = radiance(scene, Ray(mit->p, ps.wo), sampler, nv);
          auto mis = light_pdf ? balance_heuristic(ps.pdf, *light_pdf) : 1.0f;
          lo += Li * (ps.f / ps.pdf * mis);
        }
        Lo += lo * mit->W;
      }
    }

  auto Tr = transmittance(ray.o, ray.d, ray.tmax, sampler);
  if (!it) {
    if (scene.env_light) {
      Lo += Tr * scene.env_light->color(ray.d);
      if (!pv.is_delta)
        result.light_pdf = scene.env_light->pdf(ray.d);
    }
    return result;
  }

  if (it->material().is<EmissiveMaterial>()) {
    Lo += Tr * it->material().le({*it, wi});
    if (!pv.is_delta)
      result.light_pdf = light_sampler.pdf(ray, *it);
    return result;
  }

  if (pv.length + 1 >= max_path_length)
    return result;

  auto bc = BxdfSampleCtx(*it, wi, 0.6f, pv.diffuse_length > 0);
  auto bxdf = it->material().sample_bxdf(bc, sampler);

  auto beta = vec3(1.0f);
  bxdf.sample_p(beta, bc, sampler);

  auto lo = vec3(0.0f);
  if (!bxdf.is_delta()) {
    if (auto ls = light_sampler.sample(it->p, sampler);
        ls && !hit(it->spawn_ray(ls->wo, ls->distance))) {
      auto cosine = absdot(ls->wo, it->n);
      auto tr = transmittance(it->p, ls->wo, ls->distance, sampler);
      auto wo = it->to_local(ls->wo);
      if (ls->light->is_delta()) {
        auto f = bxdf.f(wo);
        lo += ls->le * tr * cosine * f / ls->pdf;
      } else {
        auto f = bxdf.f(wo);
        auto mis = balance_heuristic(ls->pdf, bxdf.pdf(wo));
        lo += ls->le * tr * cosine * f / ls->pdf * mis;
      }
    }
  }
  if (auto bs = bxdf.sample(bc, sampler)) {
    auto cosine = absdot(bs->wo, it->n);
    auto nv = Vertex(pv, bs->pdf, bs->is_delta);
    auto [Li, light_pdf] = radiance(scene, it->spawn_ray(bs->wo), sampler, nv);
    auto mis = light_pdf ? balance_heuristic(bs->pdf, *light_pdf) : 1.0f;
    lo += Li * bs->f * (cosine / bs->pdf * mis);
  }
  Lo += min(Tr * beta * lo, vec3(8));

  return result;
}

}  // namespace pine