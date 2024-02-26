#include <pine/impl/integrator/path.h>
#include <pine/core/scene.h>

namespace pine {

PathIntegrator::PathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                               int max_path_length)
    : RayIntegrator{psl::move(accel), psl::move(sampler)},
      light_sampler{psl::move(light_sampler)},
      max_path_length{max_path_length} {
  if (max_path_length <= 0)
    Fatal("`PathIntegrator` expect `max_path_length` to be positive, get", max_path_length);
}

vec3 PathIntegrator::radiance(Scene& scene, Ray ray, Interaction it, bool is_hit,
                              Sampler& sampler) {
  auto L = vec3{0.0f};
  auto beta = vec3{1.0f};
  auto prev_n = vec3{};
  auto prev_is_delta = false;
  auto bsdf_pdf = 0.0f;

  for (int depth = 0; depth < max_path_length; depth++) {
    if (depth != 0)
      is_hit = intersect(ray, it);

    if (!is_hit) {
      if (scene.env_light) {
        auto le = scene.env_light->color(ray.d);
        if (depth == 0 || prev_is_delta) {
          L += beta * le;
        } else {
          auto pdf = scene.env_light->pdf(prev_n, ray.d);
          auto mis = balance_heuristic(bsdf_pdf, pdf);
          L += beta * le * mis;
        }
      }
      break;
    }

    if (it.material()->is<EmissiveMaterial>()) {
      if (depth == 0 || prev_is_delta) {
        L += beta * it.material()->le(LeEvalCtx{it, -ray.d});
      } else {
        auto lec = LeEvalCtx(it, -ray.d);
        auto le = it.material()->le(lec);
        auto mis = 1.0f;
        auto pdf = light_sampler.pdf(it.geometry, it, ray, prev_n);
        mis = balance_heuristic(bsdf_pdf, pdf);
        L += beta * le * mis;
      }
      break;
    }

    if (depth + 1 == max_path_length)
      break;

    if (!it.material()->is_delta())
      if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
        if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
          auto cosine = absdot(ls->wo, it.n);
          auto [f, pdf] = it.material()->f_pdf({it, -ray.d, ls->wo});
          auto mis = 1.0f;
          if (!ls->light->is_delta())
            mis = balance_heuristic(ls->pdf, pdf) / 0.2f;
          L += beta * cosine * ls->le / ls->pdf * f * mis;
        }
      }

    auto msc = MaterialSampleCtx(it, -ray.d, sampler.get1d(), sampler.get2d());

    if (auto bs = it.material()->sample(msc)) {
      beta *= absdot(bs->wo, it.n) * bs->f / bs->pdf;
      bsdf_pdf = bs->pdf;
      prev_n = it.n;
      prev_is_delta = it.material()->is_delta();

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