#include <pine/core/parallel.h>
#include <pine/core/profiler.h>
#include <pine/core/scene.h>
#include <pine/impl/integrator/mlt.h>

namespace pine {

MltIntegrator::MltIntegrator(Accel accel, int avg_spp, LightSampler light_sampler,
                             int max_path_length)
    : RTIntegrator{MOVE(accel), MltSampler(0.02f, 0.02f, 0), MOVE(light_sampler)},
      avg_spp(avg_spp),
      max_path_length{max_path_length} {
  if (max_path_length <= 0)
    SEVERE("`MltIntegrator` expect `max_path_length` to be positive, get", max_path_length);
}
struct MltIntegrator::Vertex {
  Vertex(int max_path_length, int length, int diffuse_length, float pdf, bool is_delta)
      : max_path_length(max_path_length),
        length(length),
        diffuse_length(diffuse_length),
        pdf(pdf),
        is_delta(is_delta) {}
  Vertex(const Vertex& pv, float pdf, bool is_delta = false)
      : Vertex(pv.max_path_length, pv.length + 1, pv.diffuse_length + (is_delta ? 0 : 1), pdf,
               is_delta) {}
  static Vertex first(int max_path_length) { return Vertex(max_path_length, 0, 0, 0.0f, true); }
  int max_path_length;
  int length;
  int diffuse_length;
  float pdf;
  bool is_delta;
};
void MltIntegrator::render(Scene& scene) {
  RTIntegrator::render(scene);
  auto& film = scene.camera.film();

  Profiler _("[Mlt]Render");
  DEBUG("Estimating I");
  auto startup_samplers = psl::vector<Sampler>();
  auto startup_spp = psl::max(100000 / area(film.size()), 1);
  for (int i = 0; i < n_threads(); i++) startup_samplers.push_back(BlueSobolSampler(startup_spp));
  auto I = Atomic<float>();
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = startup_samplers[threadIdx].start_pixel(p, 0);
    auto L = vec3(0.0f);
    for (int si = 0; si < startup_spp; si++, sampler.start_next_sample()) {
      auto ray = scene.camera.gen_ray(vec2(p) / film.size());
      L += radiance(scene, ray, sampler, Vertex::first(max_path_length), true).Lo;
      if (L.has_nan()) LOG(p, ' ', L);
    }
    I += luminance(L);
  });
  auto Int = double(I) / startup_spp / area(film.size());

  DEBUG("Starting Markov chain");
  auto pfilms = psl::vector<vec2>(n_threads());
  auto Ls = psl::vector<vec3>(n_threads());

  for (int i = 0; i < n_threads(); i++) {
    auto pfilm = samplers[i].get2d();
    auto L = radiance(scene, scene.camera.gen_ray(pfilm), samplers[i],
                      Vertex::first(max_path_length), true)
                 .Lo;
    pfilms[i] = pfilm;
    Ls[i] = L;
  }

  auto n_total = int64_t(area(film.size())) * avg_spp;
  parallel_for(n_total, [&](int i) {
    auto& sampler = samplers[threadIdx];
    auto& m_sampler = sampler.as<MltSampler>();

    auto& pfilmp = pfilms[threadIdx];
    auto& Lp = Ls[threadIdx];

    m_sampler.start_next_sample();

    auto pfilm = m_sampler.get2d();
    auto L =
        radiance(scene, scene.camera.gen_ray(pfilm), sampler, Vertex::first(max_path_length), true)
            .Lo;

    auto l = luminance(L);
    auto lp = luminance(Lp);
    auto a = psl::min(l / lp, 1.0f);

    if (lp > 1e-6f) film.add_radiance(pfilmp * film.size(), Lp / lp * (1 - a));
    if (l > 1e-6f) film.add_radiance(pfilm * film.size(), L / l * a);
    if (sampler.randf() < a) {
      m_sampler.accept();
      pfilmp = pfilm;
      Lp = L;
    } else {
      m_sampler.reject();
    }
    if (i % 100 == 0) set_progress(float(i) / n_total / 2);
  });

  film.scale(Int / avg_spp);

  for (int i = 0; i < n_threads(); i++) startup_samplers[i] = BlueSobolSampler(avg_spp);
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = startup_samplers[threadIdx].start_pixel(p, 0);
    auto L = vec3(0.0f);
    for (int si = 0; si < avg_spp; si++, sampler.start_next_sample()) {
      auto ray = scene.camera.gen_ray((p + sampler.rand2f()) / film.size(), sampler.rand2f());
      L += radiance(scene, ray, sampler, Vertex::first(2), false).Lo;
    }
    film[p] += vec4(L / avg_spp, 1.0f);
    if (p.x % 64 == 0) set_progress(0.5f + 0.5f * progress_2d(p, film.size()));
  });
}
MltIntegrator::RadianceResult MltIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler,
                                                      Vertex pv, bool omit_direct) const {
  auto result = RadianceResult();
  auto wi = -ray.d;
  auto& Lo = result.Lo;

  auto it = intersect(ray);

  if (pv.diffuse_length == 0)
    if (auto mit = sample_medium(ray, sampler)) {
      if (pv.length + 1 < pv.max_path_length) {
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
          auto [Li, light_pdf] = radiance(scene, Ray(mit->p, ps.wo), sampler, nv, omit_direct);
          auto mis = light_pdf ? balance_heuristic(ps.pdf, *light_pdf) : 1.0f;
          lo += Li * (ps.f / ps.pdf * mis);
        }
        Lo += lo * mit->W;
      }
    }

  auto Tr = transmittance(ray.o, ray.d, ray.tmax, sampler);
  if (!it) {
    if ((!omit_direct || pv.length > 1) && scene.env_light) {
      Lo += Tr * scene.env_light->color(ray.d);
      if (!pv.is_delta) result.light_pdf = scene.env_light->pdf(ray.d);
    }
    return result;
  }

  if (it->material().is<EmissiveMaterial>()) {
    if (!omit_direct || pv.length > 1) {
      Lo += Tr * it->material().le({*it, wi});
      if (!pv.is_delta) result.light_pdf = light_sampler.pdf(ray, *it);
    }
    return result;
  }

  if (pv.length + 1 >= pv.max_path_length) return result;

  auto bc = BxdfSampleCtx(*it, wi, 0.0f, pv.diffuse_length > 0);
  auto bxdf = it->material().sample_bxdf(bc, sampler);

  auto beta = vec3(1.0f);
  bxdf.sample_p(beta, bc, sampler);

  auto lo = vec3(0.0f);
  auto pp = omit_direct ? 0.5f : 1.0f;
  if ((!omit_direct || pv.length > 0) && !bxdf.is_delta() && sampler.get1d() < pp) {
    if (auto ls = light_sampler.sample(it->p, sampler);
        ls && !hit(it->spawn_ray(ls->wo, ls->distance))) {
      auto cosine = absdot(ls->wo, it->n);
      auto tr = transmittance(it->p, ls->wo, ls->distance, sampler);
      auto wo = it->to_local(ls->wo);
      if (ls->light->is_delta()) {
        auto f = bxdf.f(wo);
        lo += ls->le * tr * cosine * f / ls->pdf / pp;
      } else {
        auto f = bxdf.f(wo);
        auto mis = balance_heuristic(ls->pdf, bxdf.pdf(wo));
        lo += ls->le * tr * cosine * f / ls->pdf * mis / pp;
      }
    }
  }
  if (sampler.get1d() < pp)
    if (auto bs = bxdf.sample(bc, sampler)) {
      auto cosine = absdot(bs->wo, it->n);
      auto nv = Vertex(pv, bs->pdf, bs->is_delta);
      auto [Li, light_pdf] = radiance(scene, it->spawn_ray(bs->wo), sampler, nv, omit_direct);
      auto mis = light_pdf ? balance_heuristic(bs->pdf, *light_pdf) : 1.0f;
      lo += Li * bs->f * (cosine / bs->pdf * mis) / pp;
    }
  Lo += Tr * beta * lo;

  return result;
}

}  // namespace pine