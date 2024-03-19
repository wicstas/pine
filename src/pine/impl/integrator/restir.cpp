// #include <pine/impl/integrator/restir.h>
// #include <pine/core/profiler.h>
// #include <pine/core/parallel.h>
// #include <pine/core/scene.h>

// namespace pine {

// RestirIntegrator::RestirIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
//                                    int max_path_length)
//     : RTIntegrator{psl::move(accel), psl::move(sampler)},
//       light_sampler{psl::move(light_sampler)},
//       max_path_length{max_path_length} {
//   if (max_path_length <= 0)
//     Fatal("`RestirIntegrator` expect `max_path_length` to be positive, get", max_path_length);
// }
// struct RestirIntegrator::Vertex {
//   Vertex(vec2i q, int length, Interaction it, float pdf, bool is_delta = false)
//       : q(q), length(length), it(psl::move(it)), pdf(pdf), is_delta(is_delta) {
//   }
//   static Vertex first_vertex(vec2i q) {
//     return Vertex(q, 0, {}, 0.0f, true);
//   }
//   vec2i q;
//   int length;
//   Interaction it;
//   float pdf;
//   bool is_delta;
// };
// struct ReSample {
//   vec3 xv, nv;
//   vec3 xs, ns;
//   vec3 Lo;
// };
// struct Reservoir {
//   void update(ReSample s_new, float w_new, float u) {
//     w += w_new;
//     M += 1;
//     if (u < w_new / w)
//       s = s_new;
//   }

//   void merge(Reservoir r, float p_hat, float u) {
//     if (r.s) {
//       auto M0 = M;
//       update(*r.s, p_hat * r.W * r.M, u);
//       M = M0 + r.M;
//     }
//   }

//   psl::optional<ReSample> s;
//   float w = 0.0f;
//   int M = 0;
//   float W = 0.0f;
// };
// static Array2d<ReSample> initial_samples;
// static Array2d<Reservoir> temporal_reservoir_back;
// static Array2d<Reservoir> temporal_reservoir;
// static Array2d<Reservoir> spatial_reservoir;
// static Array2d<Reservoir> spatial_reservoir_back;
// static int iteration = 0;
// void RestirIntegrator::render(Scene& scene) {
//   RTIntegrator::render(scene);
//   light_sampler.build(&scene);
//   auto& film = scene.camera.film();

//   Profiler _("[Restir]Render");
//   initial_samples = Array2d<ReSample>(film.size());
//   temporal_reservoir = Array2d<Reservoir>(film.size());
//   spatial_reservoir = Array2d<Reservoir>(film.size());

//   parallel_for(film.size(), [&](vec2i p) {
//     auto ray = scene.camera.gen_ray((p + vec2(0.5f)) / film.size(), vec2(0.5f));
//     if (auto it = SurfaceInteraction(); intersect(ray, it)) {
//       initial_samples[p].xv = it.p;
//       initial_samples[p].nv = it.n;
//     }
//   });

//   for (int iter = 0; iter < spp; iter++) {
//     iteration = iter;
//     temporal_reservoir_back = temporal_reservoir;
//     spatial_reservoir_back = spatial_reservoir;
//     for (auto& s : initial_samples) {
//       s.Lo = {};
//       s.ns = {};
//       s.xs = {};
//     }
//     parallel_for(film.size(), [&](vec2i p) {
//       Sampler& sampler = samplers[threadIdx].start_pixel(p, iter);
//       auto ray = scene.camera.gen_ray((p + sampler.get2d()) / film.size(), sampler.get2d());
//       auto L = radiance(scene, ray, sampler, Vertex::first_vertex(p)).Lo;
//       film.add_sample(p, L, (iter + 1) / 10.0f);
//     });
//     set_progress(float(iter) / spp);
//   }
// }
// RestirIntegrator::RadianceResult RestirIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler,
//                                                             Vertex pv) {
//   auto result = RadianceResult();
//   auto wi = -ray.d;
//   auto& Lo = result.Lo;

//   if (auto ittr = intersect_tr(ray, sampler); !ittr) {
//     if (scene.env_light) {
//       Lo += scene.env_light->color(ray.d);
//       if (!pv.is_delta)
//         result.light_pdf = scene.env_light->pdf(pv.it, ray.d);
//     }
//     return result;
//   } else if (ittr->is<MediumInteraction>()) {
//     if (pv.length + 1 < max_path_length) {
//       auto& ms = ittr->as<MediumInteraction>();
//       if (auto ls = light_sampler.sample(ms, sampler.get1d(), sampler.get2d())) {
//         if (!hit(Ray(ms.p, ls->wo, 0.0f, ls->distance))) {
//           auto tr = transmittance(ms.p, ls->wo, ls->distance, sampler);
//           auto f = ms.pg.f(wi, ls->wo);
//           if (ls->light->is_delta()) {
//             Lo += ls->le * ms.sigma * tr * f / ls->pdf;
//           } else {
//             auto mis = balance_heuristic(ls->pdf, ms.pg.pdf(wi, ls->wo));
//             Lo += ls->le * ms.sigma * tr * f / ls->pdf * mis;
//           }
//         }
//       }
//       auto ps = ms.pg.sample(wi, sampler.get2d());
//       auto nv = Vertex(pv.q, pv.length + 1, *ittr, ps.pdf);
//       auto rr = pv.length <= 1 ? 1.0f : psl::max(ms.sigma * luminance(ps.f) / ps.pdf, 0.05f);
//       if (rr >= 1 || sampler.get1d() < rr) {
//         auto [Li, light_pdf] = radiance(scene, Ray(ms.p, ps.wo), sampler, nv);
//         auto mis = light_pdf ? balance_heuristic(ps.pdf, *light_pdf) : 1.0f;
//         Lo += Li * (ms.sigma * ps.f / ps.pdf * mis / psl::min(1.0f, rr));
//       }
//     }
//     return result;
//   } else {
//     auto& it = ittr->as<SurfaceInteraction>();
//     it.n = face_same_hemisphere(it.n, wi);
//     if (it.material()->is<EmissiveMaterial>()) {
//       Lo += it.material()->le({it, wi});
//       if (!pv.is_delta)
//         result.light_pdf = light_sampler.pdf(pv.it, it, ray);
//       return result;
//     }

//     if (pv.length + 1 >= max_path_length)
//       return result;

//     if (!it.material()->is_delta()) {
//       if (auto ls = light_sampler.sample(*ittr, sampler.get1d(), sampler.get2d())) {
//         if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
//           auto cosine = absdot(ls->wo, it.n);
//           auto tr = transmittance(it.p, ls->wo, ls->distance, sampler);
//           if (ls->light->is_delta()) {
//             auto f = it.material()->f({it, wi, ls->wo});
//             Lo += ls->le * tr * cosine * f / ls->pdf;
//           } else {
//             auto [f, bsdf_pdf] = it.material()->f_pdf({it, wi, ls->wo});
//             auto mis = balance_heuristic(ls->pdf, bsdf_pdf);
//             Lo += ls->le * tr * cosine * f / ls->pdf * mis;
//           }
//         }
//       }
//     }

//     auto indirect = vec3(0.0f);
//     // if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
//     auto bs = psl::optional<BSDFSample>(BSDFSample());
//     CHECK(bs);
//     bs->wo = uniform_sphere(sampler.get2d());
//     if (dot(bs->wo, face_same_hemisphere(it.n, wi)) < 0)
//       bs->wo *= -1;
//     bs->f = it.material()->albedo({it.p, it.n, it.uv}) / Pi;
//     bs->pdf = 1.0f / (2 * Pi);
//     auto cosine = absdot(bs->wo, it.n);
//     auto nv = Vertex(pv.q, pv.length + 1, *ittr, bs->pdf, it.material()->is_delta());
//     auto rr = pv.length <= 1 ? 1.0f : psl::max(luminance(cosine * bs->f / bs->pdf), 0.05f);
//     if (rr >= 1 || sampler.get1d() < rr) {
//       auto [Li, light_pdf] = radiance(scene, it.spawn_ray(bs->wo), sampler, nv);
//       auto mis = light_pdf ? balance_heuristic(bs->pdf, *light_pdf) : 1.0f;
//       indirect += Li * bs->f * (cosine / bs->pdf * mis / psl::min(1.0f, rr));
//     }
//     // }
//     if (pv.length == 1) {
//       initial_samples[pv.q].Lo = Lo;
//       initial_samples[pv.q].xs = it.p;
//       initial_samples[pv.q].ns = it.n;
//     }
//     if (pv.length == 0) {
//       auto& S = initial_samples[pv.q];
//       auto& R = temporal_reservoir[pv.q];
//       auto w = luminance(S.Lo) / (1 / (2 * Pi));
//       if (w > 1e-6f) {
//         R.update(S, w, sampler.get1d());
//         R.W = R.w / (R.M * psl::max(epsilon, luminance(R.s->Lo)));
//       }
//       // if (R.s) {
//       //   auto wo = normalize(R.s->xs - it.p);
//       // Lo += R.W * R.s->Lo * absdot(wo, it.n) * it.material()->f({it, wi, wo});
//       // }

//       if (iteration > 0) {
//         auto& Rs = spatial_reservoir[pv.q];
//         auto Z = 0.0f;
//         if (Rs.s && luminance(Rs.s->Lo) > 0.0f)
//           Z += Rs.M;
//         for (int i = 0; i < 9; i++) {
//           auto qn = pv.q + vec2i((sampler.get2d() - vec2(0.5f)) * 36);
//           qn = clamp(qn, vec2i(0), spatial_reservoir.size() - vec2i(1));
//           if (i == 0)
//             qn = pv.q;
//           else if (qn == pv.q)
//             continue;
//           if (dot(S.nv, initial_samples[qn].nv) < 0.95f)
//             continue;
//           const auto& Rn = temporal_reservoir_back[qn];
//           if (!Rn.s)
//             continue;
//           float dvr, dvq;
//           auto vr = normalize(S.xv - Rn.s->xs, dvr);
//           auto vq = normalize(Rn.s->xv - Rn.s->xs, dvq);
//           auto J = absdot(vr, Rn.s->ns) / absdot(vq, Rn.s->ns) * dvq * dvq / (dvr * dvr);
//           if (!(J > 1e-6f))
//             continue;
//           auto p_hat_prime = luminance(Rn.s->Lo) / J;
//           Rs.merge(Rn, p_hat_prime, sampler.get1d());
//           if (Rs.s && luminance(Rs.s->Lo) > 0.0f)
//             Z += Rn.M;
//         }
//         Rs.W = Rs.w / (Z * luminance(Rs.s->Lo));

//         auto wo = normalize(Rs.s->xs - it.p);
//         Lo += Rs.W * Rs.s->Lo * absdot(wo, it.n) * it.material()->f({it, wi, wo});
//       }
//     } else {
//       Lo += indirect;
//     }
//     return result;
//   }
// }

// }  // namespace pine