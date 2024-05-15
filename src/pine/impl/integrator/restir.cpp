#include <pine/impl/integrator/restir.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>

namespace pine {

RestirIntegrator::RestirIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                                   int max_path_length)
    : RTIntegrator{MOVE(accel), MOVE(sampler)},
      light_sampler{MOVE(light_sampler)},
      max_path_length{max_path_length} {
  if (max_path_length <= 0)
    Fatal("`RestirIntegrator` expect `max_path_length` to be positive, get", max_path_length);
}
struct RestirIntegrator::Vertex {
  Vertex(vec2i q, int length, Interaction it, float pdf, bool is_delta = false,
         bool inside_subsurface = false)
      : q(q),
        length(length),
        it(MOVE(it)),
        pdf(pdf),
        is_delta(is_delta),
        inside_subsurface(inside_subsurface) {
  }
  static Vertex first_vertex(vec2i q) {
    return Vertex(q, 0, {}, 0.0f, true);
  }
  vec2i q;
  int length;
  Interaction it;
  float pdf;
  bool is_delta;
  bool inside_subsurface;
};
struct RestirIntegrator::RadianceResult {
  vec3 Lo;
  vec3 Lo_direct;
  psl::optional<float> light_pdf;
};
struct ReSample {
  float target_pdf() const {
    return average(Lo);
  }

  bool valid = false;
  vec3 xv, nv;
  float pdf = 0.0f;
  vec3 xs, ns;
  vec3 Lo;
  float t = float_max;
};
struct Reservoir {
  void update(ReSample s_new, float w_new, float u) {
    w += w_new;
    M += 1;
    if (u < w_new / w)
      z = s_new;
  }

  void merge(Reservoir r, float p_hat, float u) {
    auto M0 = M;
    update(r.z, p_hat * r.W * r.M, u);
    M = M0 + r.M;
  }

  ReSample z;
  float w = 0.0f;
  int M = 0;
  float W = 0.0f;
};
static Array2d<ReSample> initial_samples;
static Array2d<Reservoir> temporal_reservior;
static Array2d<Reservoir> spatial_reservior;

void RestirIntegrator::render(Scene& scene) {
  RTIntegrator::render(scene);
  light_sampler.build(&scene);
  auto& film = scene.camera.film().clear();

  initial_samples.resize(film.size());
  temporal_reservior.resize(film.size());
  spatial_reservior.resize(film.size());

  Profiler _("[Restir]Render");
  for (int si = 0; si < spp; si++) {
    parallel_for(film.size(), [&](vec2i p) {
      auto& S = initial_samples[p] = {};
      Sampler& sampler = samplers[threadIdx].start_pixel(p, si);
      auto ray = scene.camera.gen_ray((p + vec2(0.5f)) / film.size(), sampler.rand2f());
      auto rad = radiance(scene, ray, sampler, Vertex::first_vertex(p));
      film[p] += vec4(rad.Lo_direct / spp, 0.0f);
      auto& R = temporal_reservior[p];
      if (S.valid) {
        auto w = S.target_pdf() / S.pdf;
        R.update(S, w, sampler.randf());
      } else {
        R.M += 1;
      }
      R.W = R.w / (R.M * R.z.target_pdf());

      if (p.x % 64 == 0)
        set_progress(progress_2d(p, film.size()) / spp + float(si) / spp);
    });
    parallel_for(film.size(), [&](vec2i q) {
      Sampler& sampler = samplers[threadIdx];
      auto& Rs = spatial_reservior[q];
      auto Z = Rs.M;
      for (int i = 0; i < 16; i++) {
        auto qn = vec2i(q + (sampler.rand2f() - vec2(0.5f)) * 32);
        qn = clamp(qn, vec2i(0), film.size() - vec2i(1));
        if (dot(initial_samples[q].nv, initial_samples[qn].nv) < 0.995f)
          continue;
        if (psl::abs(initial_samples[q].t - initial_samples[qn].t) > 0.1f)
          continue;
        auto Rn = temporal_reservior[qn];
        if (Rn.z.valid) {
          //   auto vr = initial_samples[q].xv - Rn.z.xs;
          //   auto vq = initial_samples[qn].xv - Rn.z.xs;
          //   auto nq = Rn.z.ns;
          //   auto jac = psl::max(absdot(normalize(vr), nq) / absdot(normalize(vq), nq) *
          //                           length_squared(vq) / length_squared(vr),
          //                       0.01f);
          auto p_hat = Rn.z.target_pdf();
          if (hit(spawn_ray(initial_samples[q].xv, initial_samples[q].nv,
                            normalize(Rn.z.xs - initial_samples[q].xv),
                            length(Rn.z.xs - initial_samples[q].xv)))) {
            p_hat = 0.0f;
          }
          Rs.merge(Rn, p_hat, sampler.randf());
          if (p_hat > 0)
            Z += Rn.M;
        }
      }
      Rs.W = Rs.w / (Z * Rs.z.target_pdf());
    });
  }
  parallel_for(film.size(), [&](vec2i p) {
    const auto& R = spatial_reservior[p];
    if (!R.z.valid)
      return;
    Sampler& sampler = samplers[threadIdx].start_pixel(p, 0);
    auto ray = scene.camera.gen_ray((p + vec2(0.5f)) / film.size(), sampler.rand2f());
    auto wi = -ray.d, wo = normalize(R.z.xs - R.z.xv);
    auto [mit, it] = intersect_tr(ray, sampler);
    if (it && !it->material().is_delta(*it)) {
      auto cosine = absdot(wo, it->n);
      auto f = it->material().f({*it, wi, wo});
      auto Lo_indirect = R.z.Lo * R.W * cosine * f;
      film[p] += vec4(Lo_indirect, 0.0f);
    }
  });
}
RestirIntegrator::RadianceResult RestirIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler,
                                                            Vertex pv) {
  auto result = RadianceResult();
  auto wi = -ray.d;
  auto& Lo = result.Lo;

  auto [mit, it] = intersect_tr(ray, sampler);

  auto Tr = transmittance(ray.o, ray.d, ray.tmax, sampler);
  if (!it) {
    if (scene.env_light) {
      Lo += Tr * scene.env_light->color(ray.d);
      if (!pv.is_delta)
        result.light_pdf = scene.env_light->pdf(pv.it, ray.d);
    }
    return result;
  }

  if (pv.length == 1) {
    initial_samples[pv.q].xs = it->p;
    initial_samples[pv.q].ns = it->n;
    initial_samples[pv.q].valid = true;
  }

  if (it->material().is<EmissiveMaterial>()) {
    Lo += Tr * it->material().le({*it, wi});
    if (!pv.is_delta)
      result.light_pdf = light_sampler.pdf(pv.it, *it, ray);
    return result;
  }

  if (pv.length + 1 >= max_path_length)
    return result;

  if (pv.length == 0) {
    initial_samples[pv.q].xv = it->p;
    initial_samples[pv.q].nv = it->n;
    initial_samples[pv.q].t = ray.tmax;
  }

  auto lo = vec3(0.0f);
  auto rr_m = average(Tr);
  if (with_probability(1 - rr_m, sampler))
    return result;

  if (!it->material().is_delta(*it)) {
    if (auto ls = light_sampler.sample(*it, sampler.get1d(), sampler.get2d())) {
      if (!hit(it->spawn_ray(ls->wo, ls->distance))) {
        auto cosine = absdot(ls->wo, it->n);
        auto tr = transmittance(it->p, ls->wo, ls->distance, sampler);
        if (ls->light->is_delta()) {
          auto f = it->material().f({*it, wi, ls->wo, pv.is_delta ? 0.0f : 0.2f});
          lo += ls->le * tr * cosine * f / ls->pdf;
        } else {
          auto [f, bsdf_pdf] = it->material().f_pdf({*it, wi, ls->wo, pv.is_delta ? 0.0f : 0.2f});
          auto mis = balance_heuristic(ls->pdf, bsdf_pdf);
          lo += ls->le * tr * cosine * f / ls->pdf * mis;
        }
      }
    }
    result.Lo_direct = lo;
  }

  if (auto bs = it->material().sample({*it, wi, sampler.get1d(), sampler.get2d()})) {
    if (pv.length == 0)
      initial_samples[pv.q].pdf = bs->pdf;
    auto cosine = absdot(bs->wo, it->n);
    auto nv = Vertex(pv.q, pv.length + 1, *it, bs->pdf, it->material().is_delta(*it),
                     bs->enter_subsurface);
    auto albedo = luminance(cosine * bs->f / bs->pdf);
    auto rr = psl::min(pv.length <= 1 ? 1.0f : albedo, 1.0f);
    if (with_probability(rr, sampler)) {
      auto R = radiance(scene, it->spawn_ray(bs->wo), sampler, nv);
      auto mis = R.light_pdf ? balance_heuristic(bs->pdf, *R.light_pdf) : 1.0f;
      lo += R.Lo * bs->f * (cosine / bs->pdf * mis / rr);
      if (R.light_pdf || nv.is_delta)
        result.Lo_direct = lo;
      else if (pv.length == 0)
        initial_samples[pv.q].Lo = R.Lo;
    }
  }
  lo /= rr_m;
  Lo += lo * Tr;
  return result;
}

}  // namespace pine