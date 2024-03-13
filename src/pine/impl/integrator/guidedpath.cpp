#include <pine/impl/integrator/spatial_tree.h>
#include <pine/impl/integrator/guidedpath.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/denoise.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>
#include <pine/core/gui.h>

#include <algorithm>

#include <glad/glad.h>
#include <imgui/imgui.h>

namespace pine {
static SpatialTree guide;

GuidedPathIntegrator::GuidedPathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                                           int max_path_length)
    : RTIntegrator{psl::move(accel), psl::move(sampler)},
      light_sampler{psl::move(light_sampler)},
      max_path_length{max_path_length} {
  if (max_path_length <= 0)
    Fatal("`GuidedPathIntegrator` expect `max_path_length` to be positive, get", max_path_length);
}

struct IterativeScheme {
  struct Iteration {
    int number;
    int sample_index;
    int spp;
    bool is_final;
    bool prime_hit;
    float progress_start, progress_end;
  };

  IterativeScheme(int spp, int batch_size) {
    auto current_spp = 0;
    auto iter_spp = 1;
    auto prime_index = 1;

    auto is_final_iteration = false;
    for (int iter = 0; !is_final_iteration; iter++) {
      auto next_iter_spp = iter_spp;
      if (iter % batch_size == batch_size - 1)
        next_iter_spp *= 2;

      auto remaining_spp = spp - current_spp;
      if (iter_spp + next_iter_spp / 2 > remaining_spp) {
        iter_spp = remaining_spp;
        is_final_iteration = true;
      }

      auto prime_hit = iter >= prime_index * prime_index;
      iterations.emplace_back(iter, current_spp, iter_spp, is_final_iteration, prime_hit,
                              float(current_spp) / spp, float(current_spp + iter_spp) / spp);
      if (prime_hit)
        prime_index++;
      current_spp += psl::exchange(iter_spp, next_iter_spp);
    }
  }

  psl::optional<Iteration> next() {
    if (iterations.size()) {
      auto iteration = iterations.front();
      iterations.pop_front();
      return iteration;
    } else {
      return psl::nullopt;
    }
  }

private:
  psl::vector<Iteration> iterations;
};

struct OutlierRejectedVariance {
  OutlierRejectedVariance(vec2i image_size) : image_size(image_size), variances(area(image_size)) {
  }

  void set(vec2i p, vec3 sample, vec3 ref, vec3 min_relative) {
    variances[p.x + p.y * image_size.x] = psl::sqr((sample - ref) / max(ref, min_relative));
  }

  vec3 compute(float rejection_fraction) {
    std::sort(variances.begin(), variances.end(),
              [](vec3 a, vec3 b) { return average(a) < average(b); });

    auto N = area(image_size) * (1 - rejection_fraction);
    return psl::mean<vec3d>(psl::trim(variances, 0, N));
  }

private:
  vec2i image_size;
  psl::vector<vec3> variances;
};

struct GuidedPathIntegrator::Vertex {
  Vertex(int length, vec3 throughput, vec3 n, vec3 p, float pdf, bool is_delta = false)
      : length(length), throughput(throughput), n(n), p(p), pdf(pdf), is_delta(is_delta) {
  }
  static Vertex first_vertex(vec3 I) {
    return Vertex(0, vec3(1.0f) / I, vec3(0), vec3(0), 0.0f, true);
  }
  int length;
  vec3 throughput;
  vec3 n;
  vec3 p;
  float pdf;
  bool is_delta;
};

void GuidedPathIntegrator::render(Scene& scene) {
  RTIntegrator::render(scene);
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("`GuidedPathIntegrator` doesn't support `Plane`, please use `Rect` or `Disk` instead");

  light_sampler.build(&scene);
  auto& film = scene.camera.film();

  Profiler _("[GuidedPath]Render");

  auto initial_samples = area(film.size());
  auto spatial_k = 4000;
  guide = SpatialTree(scene.get_aabb(), initial_samples, spatial_k);
  auto spatial_ratio = psl::max<float>(spatial_k / psl::sqrt<float>(initial_samples), 1.0f);
  auto iteration_scheme = IterativeScheme(spp, 1);

  // For denoising later
  auto albedo = Array2d3f(film.size());
  auto normal = Array2d3f(film.size());
  auto position = Array2d3f(film.size());
  parallel_for(film.size(), [&](vec2i p) {
    auto ray = scene.camera.gen_ray((p + vec2(0.5f)) / film.size(), vec2(0.5f));
    if (auto it = SurfaceInteraction(); intersect(ray, it)) {
      albedo[p] = it.material()->albedo({it.p, it.n, it.uv});
      normal[p] = it.n;
      position[p] = it.p;
    }
  });

  auto acc_I = Array2d3f(film.size());
  auto acc_weight = 0.0f;
  auto I = Array2d3f(film.size());
  auto I_estimate = Array2d3f(film.size());
  auto I_estimate_populated = false;
  auto or_variance = OutlierRejectedVariance(film.size());

  set_progress(0);
  while (auto iter = iteration_scheme.next()) {
    collect_radiance_sample = !iter->is_final;
    use_learned_ratio = iter->number > 0 ? 0.5f : 0.0f;

    parallel_for(film.size(), [&](vec2i p) {
      auto& sampler = samplers[threadIdx].start_pixel(p, iter->sample_index);
      auto Ie = I_estimate[p];
      auto L = vec3(0.0f);
      for (int si = 0; si < iter->spp; si++) {
        auto ray = scene.camera.gen_ray((p + sampler.get2d()) / film.size(), sampler.get2d());
        auto stats = Stats();
        L += radiance(scene, ray, sampler, Vertex::first_vertex(max(Ie, vec3(1e-2f))), stats);
        sampler.start_next_sample();
      }
      I[p] = L / iter->spp;
      or_variance.set(p, L / iter->spp, Ie, vec3(0.01f));
      if (p.x % 64 == 0)
        set_progress(psl::lerp(float(p.x + p.y * film.size().x) / area(film.size()),
                               iter->progress_start, iter->progress_end));
    });

    if (!psl::exchange(I_estimate_populated, true)) {
      denoise(DenoiseQuality::High, I_estimate, I, albedo, normal);
      parallel_for(film.size(),
                   [&](vec2i p) { or_variance.set(p, I[p], I_estimate[p], vec3(0.01f)); });
    }
    auto variance = or_variance.compute(0.00001f);
    Log("variance: ", (float)average(variance));
    auto weight = 1.0f / psl::max<float>(average(variance), epsilon);
    combine_inplace(acc_I, I, acc_weight, weight);
    acc_weight += weight;
    if (iter->prime_hit)
      denoise(DenoiseQuality::High, I_estimate, acc_I, albedo, normal);

    if (!iter->is_final)
      guide.refine(spatial_ratio * psl::sqrt<float>(iter->spp * area(film.size())));
  }

  film.pixels = Array2d4f::from(acc_I);

  ///////////////////////////////////////////////////////////////////////////////
  // const int ntextures = 4;
  // launch_gui("Path Guiding", film.size() * vec2i(ntextures, 1), [&]() {
  //   static GLuint textures[ntextures];
  //   static Array2d3f images[ntextures];
  //   static bool first_run = true;
  //   static const char* window_names[]{"W0", "W1", "W2", "W3"};
  //   if (psl::exchange(first_run, false)) {
  //     psl::fill(images, Array2d3f(film.size()));
  //     glCreateTextures(GL_TEXTURE_2D, ntextures, textures);
  //     for (int i = 0; i < ntextures; i++) {
  //       glBindTexture(GL_TEXTURE_2D, textures[i]);
  //       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  //       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  //       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  //       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  //     }
  //     for_2d(film.size(), [&](vec2i p) {
  //       images[0][p] = correct_gamma(uncharted2_filmic(vec3(film[{p.x, film.height() - 1 -
  //       p.y}])));
  //     });
  //   }
  //   auto [x, y] = ImGui::GetMousePos();
  //   y = ImGui::GetMainViewport()->Size.y - 1 - y;

  //   if (psl::inside(x, 0, film.width()) && psl::inside(y, 0, film.height())) {
  //     auto& leaf = guide.traverse(position[{x, y}]);
  //     parallel_for(film.size(), [&](vec2i p) {
  //       auto pw = vec2(p) / film.size();
  //       auto w = uniform_sphere(pw);
  //       auto stats = Stats();
  //       auto& sampler = samplers[threadIdx].start_pixel(p, 0);
  //       images[1][p] = color_map_auto(leaf.li_estimate(w) / 1.5f);
  //       images[2][p] = color_map_auto(leaf.li_estimate(w, false) / 1.5f);
  //       images[3][p] = correct_gamma(
  //           uncharted2_filmic(radiance(scene, Ray(position[{x, y}], w, 1e-4f), sampler,
  //                                      Vertex::first_vertex(I_estimate[p]), stats)));
  //     });
  //   }
  //   for (int i = 0; i < ntextures; i++) {
  //     glBindTexture(GL_TEXTURE_2D, textures[i]);
  //     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, film.width(), film.height(), 0, GL_RGB, GL_FLOAT,
  //                  images[i].data());
  //   }

  //   auto window_flag = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
  //                      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings |
  //                      ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar;
  //   for (int i = 0; i < ntextures; i++) {
  //     ImGui::Begin(window_names[i], nullptr, window_flag);
  //     ImGui::SetWindowSize(ImVec2(film.width(), film.height()));
  //     ImGui::SetWindowPos(ImVec2(film.width() * i, 0));
  //     ImGui::Image((void*)(intptr_t)textures[i], ImVec2(film.width(), film.height()));
  //     ImGui::End();
  //   }
  // });
  ///////////////////////////////////////////////////////////////////////////////

  set_progress(1.0f);
}

vec3 GuidedPathIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler, Vertex pv,
                                    Stats& stats) {
  auto Lo = vec3(0.0f);
  auto wi = -ray.d;

  auto it = SurfaceInteraction();
  if (!intersect(ray, it)) {
    if (scene.env_light && pv.length == 0)
      Lo += scene.env_light->color(ray.d);
    return Lo;
  }

  if (it.material()->is<EmissiveMaterial>()) {
    if (pv.length == 0)
      Lo += it.material()->le({it, wi});
    return Lo;
  }

  if (pv.length + 1 >= max_path_length)
    return Lo;

  auto& leaf = guide.traverse(it.p);
  auto prob_a = it.material()->is_delta() ? 0.0f : use_learned_ratio;
  auto select_guide = prob_a == 1.0f || (prob_a > 0.0f && sampler.get1d() < prob_a);

  auto lo = vec3(0.0f);
  // Sample direct lighting
  if (!it.material()->is_delta())
    if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
      if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
        auto cosine = absdot(ls->wo, it.n);
        if (ls->light->is_delta()) {
          auto f = it.material()->f({it, wi, ls->wo});
          lo += ls->le * cosine * f / ls->pdf;
        } else {
          auto [f, bsdf_pdf] = it.material()->f_pdf({it, wi, ls->wo});
          auto mis = balance_heuristic(ls->pdf, bsdf_pdf);
          lo += ls->le * cosine * f / ls->pdf * mis;
        }
      }
    }
  if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
    auto nray = it.spawn_ray(bs->wo);
    if (auto nit = SurfaceInteraction(); intersect(nray, nit)) {
      if (nit.material()->is<EmissiveMaterial>()) {
        auto cosine = absdot(bs->wo, it.n);
        auto mis = 1.0f;
        if (!it.material()->is_delta()) {
          auto light_pdf = light_sampler.pdf(nit.geometry, nit, nray, it.n);
          mis = balance_heuristic(bs->pdf, light_pdf);
        }
        lo += nit.material()->le({nit, -bs->wo}) * cosine * bs->f / bs->pdf * mis;
      }
    } else if (scene.env_light) {
      auto cosine = absdot(bs->wo, it.n);
      auto mis = 1.0f;
      if (!it.material()->is_delta()) {
        auto light_pdf = scene.env_light->pdf(it.n, bs->wo);
        mis = balance_heuristic(bs->pdf, light_pdf);
      }
      lo += scene.env_light->color(bs->wo) * cosine * bs->f / bs->pdf * mis;
    }
  }

  // Sample indirect lighting
  if (select_guide) {
    if (auto gs = leaf.sample(sampler.get2d())) {
      auto cosine = absdot(gs->wo, it.n);
      auto mec = MaterialEvalCtx(it, wi, gs->wo);
      auto f = it.material()->f(mec);
      auto nv = Vertex(pv.length + 1, pv.throughput * cosine * f / gs->pdf, it.n, it.p, gs->pdf);
      auto li = radiance(scene, it.spawn_ray(gs->wo), sampler, nv, stats);
      auto mis = balance_heuristic(prob_a, gs->pdf, 1 - prob_a, it.material()->pdf(mec));
      lo += li * cosine * f / gs->pdf * mis / prob_a;
      if (collect_radiance_sample)
        guide.add_sample(leaf, it.p, RadianceSample(gs->wo, average(li) / gs->pdf * mis / prob_a),
                         sampler.get3d());
    }
  } else {
    if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
      auto cosine = absdot(bs->wo, it.n);
      auto nv = Vertex(pv.length + 1, pv.throughput * cosine * bs->f / bs->pdf, it.n, it.p, bs->pdf,
                       it.material()->is_delta());
      auto li = radiance(scene, it.spawn_ray(bs->wo), sampler, nv, stats);
      auto mis = balance_heuristic(1 - prob_a, bs->pdf, prob_a, leaf.pdf(bs->wo));
      lo += li * cosine * bs->f / bs->pdf * mis / (1 - prob_a);
      if (collect_radiance_sample && !it.material()->is_delta())
        guide.add_sample(leaf, it.p,
                         RadianceSample(bs->wo, average(li) / bs->pdf * mis / (1 - prob_a)),
                         sampler.get3d());
    }
  }
  Lo += lo;

  return Lo;
}

}  // namespace pine