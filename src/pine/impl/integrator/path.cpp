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

    if (it.material->is<EmissiveMaterial>()) {
      if (depth == 0 || prev_delta) {
        L += beta * it.material->le(LeEvalCtx{it, -ray.d});
      } else {
        auto lec = LeEvalCtx(it, -ray.d);
        auto le = it.material->le(lec);
        auto mis = 1.0f;
        auto pdf = light_sampler.pdf(it.geometry, it, ray, prev_n);
        mis = power_heuristic(1, mis_bxdf_pdf, 1, pdf);
        L += beta * le * mis;
      }
      break;
    }

    if (depth + 1 == max_depth)
      break;

    if (!it.material->is_delta())
      if (auto ls = light_sampler.sample(it.p, it.n, sampler.Get1D(), sampler.Get2D())) {
        if (!hit(it.SpawnRay(ls->wo, ls->distance))) {
          auto cosine = absdot(ls->wo, it.n);
          auto mec = MaterialEvalCtx(it, -ray.d, ls->wo);
          auto f = it.material->F(mec);
          auto pdf = it.material->pdf(mec);
          auto mis = 1.0f;
          if (!ls->light->is_delta())
            mis = power_heuristic(1, ls->pdf, 1, pdf);
          L += beta * cosine * ls->le / ls->pdf * f * mis;
        }
      }

    if (nee_env_light && scene.env_light && !it.material->is_delta()) {
      auto ls = scene.env_light->sample(it.n, sampler.Get2D());

      if (ls && !hit(it.SpawnRay(ls->wo, ls->distance))) {
        auto cosine = absdot(ls->wo, it.n);
        auto mec = MaterialEvalCtx(it, -ray.d, ls->wo);
        auto f = it.material->F(mec);
        auto pdf = it.material->pdf(mec);
        auto mis = power_heuristic(1, ls->pdf, 1, pdf);
        L += beta * cosine * ls->le / ls->pdf * f * mis;
      }
    }

    auto msc = MaterialSampleCtx(it, -ray.d, sampler.Get1D(), sampler.Get2D());

    if (auto bs = it.material->sample(msc)) {
      beta *= absdot(bs->wo, it.n) * bs->f / bs->pdf;
      mis_bxdf_pdf = bs->pdf;
      prev_n = it.n;
      prev_delta = it.material->is_delta();

      // if (depth >= 1) {
      //   auto p = psl::min((beta[0] + beta[1] + beta[2]) / 3, 1.0f);
      //   if (p < 0.2f) {
      //     if (sampler.Get1D() < p)
      //       beta /= p;
      //     else
      //       break;
      //   }
      // }

      if (beta.is_black())
        break;
      ray = it.SpawnRay(bs->wo);
    } else {
      break;
    }
  }

  return L;
}

}  // namespace pine