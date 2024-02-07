#include <pine/impl/integrator/path.h>
#include <pine/core/scene.h>

namespace pine {

vec3 PathIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler) {
  auto L = vec3{0.0f};
  auto beta = vec3{1.0f};
  auto prev_n = vec3{};
  auto prev_delta = false;
  auto mis_bxdf_pdf = 0.0f;

  for (int depth = 0; depth < max_depth; depth++) {
    auto it = Interaction{};
    if (!intersect(ray, it)) {
      if (scene.env_light) {
        auto le = scene.env_light->color(ray.d);
        if (depth == 0 || !nee_env_light || prev_delta) {
          L += beta * le;
        } else {
          auto pdf = scene.env_light->pdf(prev_n, ray.d);
          auto mis = power_heuristic(1, mis_bxdf_pdf, 1, pdf);
          L += beta * le * mis;
        }
      }
      break;
    }

    if (it.geometry->material.get()->is<EmissiveMaterial>()) {
      if (depth == 0 || prev_delta) {
        L += beta * it.geometry->material.get()->le(LeEvalCtx{it, -ray.d});
      } else {
        auto lec = LeEvalCtx(it, -ray.d);
        auto le = it.geometry->material.get()->le(lec);
        auto mis = 1.0f;
        auto pdf = light_sampler.pdf(it.geometry, it, ray, prev_n);
        mis = power_heuristic(1, mis_bxdf_pdf, 1, pdf);
        L += beta * le * mis;
      }
      break;
    }

    if (depth + 1 == max_depth)
      break;

    if (!it.geometry->material.get()->is_delta())
      if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
        if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
          auto cosine = absdot(ls->wo, it.n);
          auto mec = MaterialEvalCtx(it, -ray.d, ls->wo);
          auto f = it.geometry->material.get()->F(mec);
          auto pdf = it.geometry->material.get()->pdf(mec);
          auto mis = 1.0f;
          if (!ls->light->is_delta())
            mis = power_heuristic(1, ls->pdf, 1, pdf);
          L += beta * cosine * ls->le / ls->pdf * f * mis;
        }
      }

    if (nee_env_light && scene.env_light && !it.geometry->material.get()->is_delta()) {
      auto ls = scene.env_light->sample(it.n, sampler.get2d());

      if (ls && !hit(it.spawn_ray(ls->wo, ls->distance))) {
        auto cosine = absdot(ls->wo, it.n);
        auto mec = MaterialEvalCtx(it, -ray.d, ls->wo);
        auto f = it.geometry->material.get()->F(mec);
        auto pdf = it.geometry->material.get()->pdf(mec);
        auto mis = power_heuristic(1, ls->pdf, 1, pdf);
        L += beta * cosine * ls->le / ls->pdf * f * mis;
      }
    }

    auto msc = MaterialSampleCtx(it, -ray.d, sampler.get1d(), sampler.get2d());

    if (auto bs = it.geometry->material.get()->sample(msc)) {
      beta *= absdot(bs->wo, it.n) * bs->f / bs->pdf;
      mis_bxdf_pdf = bs->pdf;
      prev_n = it.n;
      prev_delta = it.geometry->material.get()->is_delta();

      if (beta.is_black())
        break;
      ray = it.spawn_ray(bs->wo);
    } else {
      break;
    }
  }

  return L;
}

}  // namespace pine