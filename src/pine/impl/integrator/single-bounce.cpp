#include <pine/impl/integrator/single-bounce.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>

namespace pine {

void SingleBounceIntegrator::render(Scene& scene) {
  light_sampler.build(&scene);

  accel.build(&scene);
  auto& film = scene.camera.film();
  film.clear();
  set_progress(0);

  auto primary_spp = psl::max(samples_per_pixel / primary_ratio, 1);
  Profiler _("Rendering");
  for (int i = 0; i < primary_spp; i++) {
    Atomic<int64_t> max_index = 0;
    parallel_for(film.size(), [&](vec2i p) {
      Sampler& sampler = samplers[threadIdx];
      sampler.start_pixel(p, i * primary_ratio);
      pixel_color(scene, p, sampler);
      if (p.x == 0) {
        max_index = psl::max<int64_t>(max_index, p.x + p.y * film.size().x);
        set_progress(float(i) / primary_spp + float(max_index) / area(film.size()) / primary_spp);
      }
    });
    set_progress(static_cast<float>(i + 1) / primary_spp);
  }

  set_progress(1);
}

void SingleBounceIntegrator::pixel_color(Scene& scene, vec2i p, Sampler& sampler) {
  auto p_film = vec2(p + sampler.get2d()) / scene.camera.film().size();
  auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
  auto L = radiance(scene, ray, sampler, false);
  CHECK(!L.has_nan());
  CHECK(!L.has_inf());
  scene.camera.film().add_sample(p, L);
}

vec3 SingleBounceIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler,
                                      bool indirect_bounce) {
  auto L = vec3(0.0f);
  auto it = Interaction();
  if (!intersect(ray, it)) {
    if (scene.env_light && !indirect_bounce)
      L = scene.env_light->color(ray.d);
    else if (scene.env_light && indirect_bounce)
      L = clamp(scene.env_light->color(ray.d), vec3(0), vec3(1));
    return L;
  }

  if (!indirect_bounce && it.geometry->material->is<EmissiveMaterial>()) {
    L = it.geometry->material->le(LeEvalCtx(it, -ray.d));
    return L;
  }

  auto L_direct = vec3(0.0f);
  auto L_indirect = vec3(0.0f);

  if (!it.geometry->material->is_delta()) {
    if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
      if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
        auto mec = MaterialEvalCtx(it, -ray.d, ls->wo);
        auto f = it.geometry->material->F(mec);
        auto bsdf_pdf = it.geometry->material->pdf(mec);
        auto cosine = absdot(ls->wo, it.n);
        L_direct += ls->le * cosine * f / ls->pdf * balance_heuristic(ls->pdf, bsdf_pdf);
      }
    }
    if (scene.env_light)
      if (auto ls = scene.env_light->sample(it.n, sampler.get2d())) {
        if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
          auto mec = MaterialEvalCtx(it, -ray.d, ls->wo);
          auto f = it.geometry->material->F(mec);
          auto bsdf_pdf = it.geometry->material->pdf(mec);
          auto cosine = absdot(ls->wo, it.n);
          L_direct += ls->le * cosine * f / ls->pdf * balance_heuristic(ls->pdf, bsdf_pdf);
        }
      }
  }

  if (!indirect_bounce)
    for (int sp = 0; sp < primary_ratio; sp++) {
      auto msc = MaterialSampleCtx(it, -ray.d, sampler.get1d(), sampler.get2d());
      if (auto bs = it.geometry->material->sample(msc)) {
        auto l = radiance(scene, it.spawn_ray(bs->wo), sampler, true);
        auto cosine = absdot(bs->wo, it.n);
        L_indirect += l * cosine * bs->f / bs->pdf;
      }
      sampler.start_next_sample();
    }

  L += L_direct;
  if (!indirect_bounce)
    L += L_indirect / primary_ratio;

  return L;
}

}  // namespace pine