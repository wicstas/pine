#include <pine/impl/integrator/path-openpgl.h>
#include <pine/core/sampling.h>
#include <pine/core/parallel.h>
#include <pine/core/profiler.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

#include <openpgl/cpp/OpenPGL.h>

namespace pine {

static openpgl::cpp::Field* field_ptr;
static openpgl::cpp::SampleStorage* sample_storage_ptr;
static bool use_guide = false;
static bool collect = true;

void OpenPGLIntegrator::render(Scene& scene) {
  accel.build(&scene);
  light_sampler.build(&scene);
  auto& film = scene.camera.film();
  film.clear();
  use_guide = false;
  collect = true;

  Profiler _("Rendering");
  auto device = openpgl::cpp::Device{PGL_DEVICE_TYPE::PGL_DEVICE_TYPE_CPU_8};
  auto sample_storage = openpgl::cpp::SampleStorage{};
  sample_storage_ptr = &sample_storage;
  auto arg = PGLFieldArguments{};
  pglFieldArgumentsSetDefaults(arg, PGL_SPATIAL_STRUCTURE_KDTREE,
                               PGL_DIRECTIONAL_DISTRIBUTION_QUADTREE);
  auto field = openpgl::cpp::Field{&device, arg};
  field_ptr = &field;
  auto aabb = scene.get_aabb();
  field.SetSceneBounds(pgl_box3f{{aabb.lower[0], aabb.lower[1], aabb.lower[2]},
                                 {aabb.upper[0], aabb.upper[1], aabb.upper[2]}});

  auto total_pixels = static_cast<size_t>(area(film.size()));
  auto total_samples = total_pixels * samplesPerPixel;
  auto initial_samples = size_t{1024 * 16};

  auto current_sample_index = 0;
  auto current_samples = size_t{0};
  auto current_film = film;

  set_progress(0);
  for (int iteration = 0;; iteration++) {
    auto iter_samples = initial_samples * (1 << iteration);

    if (current_samples + iter_samples >= total_samples)
      collect = false;

    {
      Profiler _("Collecting");
      auto downsize = psl::min(psl::sqrt(static_cast<double>(iter_samples) / total_pixels), 1.0);
      auto iter_size = vec2i{film.size() * downsize};
      auto iter_n_pass = iter_samples / area(iter_size);
      for (size_t si = 0; si < iter_n_pass; si++) {
        ParallelFor(iter_size, [&](vec2i p) {
          auto& sampler = samplers[threadIdx];
          sampler.StartPixel(p, current_sample_index);

          auto p_film = vec2{p + sampler.Get2D()} / iter_size;
          auto ray = scene.camera.gen_ray(p_film, sampler.Get2D());
          auto L = radiance(scene, ray, sampler, 0, 1.0f, vec3{0.0f}, true).L;
          current_film.add_sample(p_film * current_film.size(), L);
        });
        set_progress(static_cast<double>(current_samples) / total_samples);
        current_sample_index += 1;
        current_samples += area(iter_size);
      }

      film = combine(film, current_film, iteration == 0 ? 0 : 1, 2);
      if (current_samples >= total_samples)
        break;
      current_film.clear();
    }
    {
      Profiler _("Refinement");
      use_guide = true;
      if (collect) {
        field.Update(sample_storage);
        field.Validate();
      }
    }
  }
  set_progress(1.0f);
}

OpenPGLIntegrator::RadianceResult OpenPGLIntegrator::radiance(Scene& scene, Ray ray,
                                                              Sampler& sampler, int depth,
                                                              float prev_sample_pdf, vec3 prev_n,
                                                              bool prev_delta) {
  auto wi = -ray.d;
  auto it = Interaction{};
  if (!intersect(ray, it)) {
    return RadianceResult{vec3{0.0f}, 0.0f};
  }

  if (it.material->is<EmissiveMaterial>()) {
    auto le = it.material->le({it, -wi});
    if (prev_delta) {
      return {le, ray.tmax};
    } else {
      auto light_pdf = light_sampler.pdf(it.geometry, it, ray, prev_n);
      auto mis_factor = power_heuristic(1, prev_sample_pdf, 1, light_pdf);
      return {le * mis_factor, ray.tmax};
    }
  }

  if (depth + 1 == max_depth)
    return {vec3{0.0f}, ray.tmax};

  auto lo = vec3{0.0f};

  auto ssd = openpgl::cpp::SurfaceSamplingDistribution{field_ptr};
  auto sample_1d = sampler.Get1D();
  ssd.Init(field_ptr, {it.p[0], it.p[1], it.p[2]}, sample_1d);
  ssd.ApplyCosineProduct({it.n[0], it.n[1], it.n[2]});

  auto guide_select_prob = use_guide ? 0.95f : 0.0f;
  if (it.material->is_delta())
    guide_select_prob = 0.0f;
  if (sampler.Get1D() < guide_select_prob) {
    auto sample_2d = sampler.Get2D();
    auto wo_p = pgl_vec3f{};
    auto guide_pdf = ssd.SamplePDF({sample_2d[0], sample_2d[1]}, wo_p);
    auto wo = vec3{wo_p.x, wo_p.y, wo_p.z};
    auto cosine = absdot(it.n, wo);
    auto mec = MaterialEvalCtx{it, wi, wo};
    auto f = it.material->F(mec);
    if (psl::abs(guide_pdf) > 1e-8f && cosine > 1e-8f && length_squared(f) > 1e-12f) {
      auto bsdf_pdf = it.material->pdf(mec);
      auto [li, dist] =
          radiance(scene, it.SpawnRay(wo), sampler, depth + 1, guide_pdf, it.n, false);
      auto mis_factor = power_heuristic(1, guide_pdf, 1, bsdf_pdf) / guide_select_prob;
      lo += li * cosine * f / guide_pdf * mis_factor;
      auto sd = openpgl::cpp::SampleData{};
      sd.direction = wo_p;
      sd.position = {it.p[0], it.p[1], it.p[2]};
      sd.distance = dist;
      sd.pdf = guide_pdf;
      sd.weight = length(li) / guide_pdf;
      sd.flags = 0;
      sample_storage_ptr->AddSample(sd);
    }

    if (auto ls = light_sampler.sample(it.p, it.n, sampler.Get1D(), sampler.Get2D())) {
      if (ls->pdf > 1e-8f && !hit(it.SpawnRay(ls->wo, ls->distance))) {
        auto mec = MaterialEvalCtx{it, wi, ls->wo};
        auto f = it.material->F(mec);
        auto cosine = absdot(it.n, ls->wo);
        auto mis_factor =
            power_heuristic(1, ls->pdf, 1, ssd.PDF({ls->wo[0], ls->wo[1], ls->wo[2]}));
        lo += ls->le / ls->pdf * cosine * f * mis_factor;
        auto sd = openpgl::cpp::SampleData{};
        sd.direction = {ls->wo[0], ls->wo[1], ls->wo[2]};
        sd.position = {it.p[0], it.p[1], it.p[2]};
        sd.distance = ls->distance;
        sd.pdf = ls->pdf;
        sd.weight = length(ls->le) / ls->pdf;
        sd.flags = 0;
        sample_storage_ptr->AddSample(sd);
      }
    }
  } else {
    if (auto bs = it.material->sample({it, wi, sampler.Get1D(), sampler.Get2D()})) {
      auto cosine = absdot(it.n, bs->wo);
      auto guide_pdf = ssd.PDF({bs->wo[0], bs->wo[1], bs->wo[2]});
      if (!psl::isnan(guide_pdf) && psl::abs(bs->pdf) > 1e-8f && cosine > 1e-8f &&
          length_squared(bs->f) > 1e-12f) {
        auto [li, dist] = radiance(scene, it.SpawnRay(bs->wo), sampler, depth + 1, bs->pdf, it.n,
                                   it.material->is_delta());
        auto mis_factor = power_heuristic(1, bs->pdf, 1, guide_pdf) / (1 - guide_select_prob);
        lo += li * cosine * bs->f / bs->pdf * mis_factor;
        auto sd = openpgl::cpp::SampleData{};
        sd.direction = {bs->wo[0], bs->wo[1], bs->wo[2]};

        sd.position = {it.p[0], it.p[1], it.p[2]};
        sd.distance = dist;
        sd.pdf = bs->pdf;
        sd.weight = length(li) / bs->pdf;
        sd.flags = 0;
        sample_storage_ptr->AddSample(sd);
      }
    }
    if (!it.material->is_delta())
      if (auto ls = light_sampler.sample(it.p, it.n, sampler.Get1D(), sampler.Get2D())) {
        if (ls->pdf > 1e-8f && !hit(it.SpawnRay(ls->wo, ls->distance))) {
          auto mec = MaterialEvalCtx{it, wi, ls->wo};
          auto f = it.material->F(mec);
          auto cosine = absdot(it.n, ls->wo);
          auto mis_factor =
              power_heuristic(1, ls->pdf, 1, ssd.PDF({ls->wo[0], ls->wo[1], ls->wo[2]}));
          lo += ls->le / ls->pdf * cosine * f * mis_factor;
          auto sd = openpgl::cpp::SampleData{};
          sd.direction = {ls->wo[0], ls->wo[1], ls->wo[2]};
          sd.position = {it.p[0], it.p[1], it.p[2]};
          sd.distance = ls->distance;
          sd.pdf = ls->pdf;
          sd.weight = length(ls->le) / ls->pdf;
          sd.flags = 0;
          sample_storage_ptr->AddSample(sd);
        }
      }
  }

  return {lo, ray.tmax};
}

}  // namespace pine