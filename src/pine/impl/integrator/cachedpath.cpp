#include <pine/impl/integrator/cachedpath.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/denoise.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

#include <psl/array.h>

namespace pine {

namespace {

struct DirectionalBin {
  void add_flux(vec3 l) {
    flux += l;
    nsamples += 1;
  }
  vec3 flux_estimate() const {
    if (nsamples != 0)
      return flux / nsamples;
    else
      return vec3(0);
  }

private:
  Vector3<Atomic<float>> flux;
  Atomic<uint32_t> nsamples;
};

struct SpatialNode {
  DirectionalBin& bin_of(vec3 w) {
    auto sc = inverse_uniform_sphere(w);
    auto x = psl::min(int(sc.x * bin_resolution), bin_resolution - 1);
    auto y = psl::min(int(sc.y * bin_resolution), bin_resolution - 1);
    return bins[x + y * bin_resolution];
  }
  const DirectionalBin& bin_of(vec3 w) const {
    return const_cast<SpatialNode*>(this)->bin_of(w);
  }

  static constexpr int bin_resolution = 2;
  psl::Array<DirectionalBin, bin_resolution * bin_resolution> bins;
};

struct SpatialTree {
  SpatialTree() = default;
  SpatialTree(AABB aabb, vec3i64 resolution) : tight_aabb(aabb), aabb(aabb), grid(resolution) {
    this->aabb.extend_by(1e-4f);
  }
  void add_sample(vec3 p, vec3 w, vec3 flux) {
    bin_of(p, w).add_flux(flux);
  }
  vec3 flux_estimate(vec3 p, vec3 w, vec3 up) const {
    p += (up - vec3(0.5f)) * tight_aabb.diagonal() / grid.size();
    p = clamp(p, tight_aabb.lower, tight_aabb.upper);
    return bin_of(p, w).flux_estimate();
  }

private:
  DirectionalBin& bin_of(vec3 p, vec3 w) {
    return grid[aabb.relative_position(p) * grid.size()].bin_of(w);
  }
  const DirectionalBin& bin_of(vec3 p, vec3 w) const {
    return const_cast<SpatialTree*>(this)->bin_of(p, w);
  }

  AABB tight_aabb;
  AABB aabb;
  Array3d<SpatialNode> grid;
};

}  // namespace

static SpatialTree stree;

CachedPathIntegrator::CachedPathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                                           int max_path_length, int max_axis_resolution,
                                           int starting_depth)
    : RTIntegrator{MOVE(accel), MOVE(sampler)},
      light_sampler{MOVE(light_sampler)},
      max_path_length{max_path_length},
      max_axis_resolution(max_axis_resolution),
      starting_depth{starting_depth} {
  if (max_path_length <= 0)
    Fatal("`CachedPathIntegrator` expect `max_path_length` to be positive, get", max_path_length);
  if (max_axis_resolution <= 0)
    Fatal("`CachedPathIntegrator` expect `max_axis_resolution` to be positive, get",
          max_axis_resolution);
  if (starting_depth < 0)
    Fatal("`CachedPathIntegrator` expect `starting_depth` to be non-negative, get", starting_depth);
}
struct CachedPathIntegrator::Vertex {
  Vertex(int length, Interaction it, float pdf, bool is_delta = false,
         bool inside_subsurface = false)
      : length(length),
        it(MOVE(it)),
        pdf(pdf),
        is_delta(is_delta),
        inside_subsurface(inside_subsurface) {
  }
  static Vertex first_vertex() {
    return Vertex(0, {}, 0.0f, true);
  }
  int length;
  Interaction it;
  float pdf;
  bool is_delta;
  bool inside_subsurface = false;
};
void CachedPathIntegrator::render(Scene& scene) {
  RTIntegrator::render(scene);
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("CachedPathIntegrator doesn't support `Plane`, please use `Rect` or `Disk` instead");

  light_sampler.build(&scene);
  auto& film = scene.camera.film();
  auto aabb = scene.get_aabb();
  auto resolution = vec3i(max_axis_resolution * aabb.diagonal() / max_value(aabb.diagonal()));
  resolution = max(resolution, vec3i(1, 1, 1));
  footprint = max_value(aabb.diagonal()) / max_axis_resolution;
  stree = SpatialTree(aabb, resolution);
  auto image0 = Array2d3f(film.size());
  auto image1 = Array2d3f(film.size());

  Profiler _("[CachedPath]Render");

  set_progress(0.0f);

  auto albedo = Array2d3f(film.size());
  auto normal = Array2d3f(film.size());
  parallel_for(film.size(), [&](vec2i p) {
    auto ray = scene.camera.gen_ray((p + vec2(0.5f)) / film.size(), vec2(0.5f));
    if (auto it = SurfaceInteraction(); intersect(ray, it)) {
      albedo[p] = it.material().albedo(it);
      normal[p] = it.n;
    }
  });

  auto learning_spp = psl::max(spp / 4, 1);
  auto rendering_spp = psl::max(spp - learning_spp, 1);

  learning_phase = true;
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = samplers[threadIdx].start_pixel(p, 0);
    auto L = vec3(0.0f);
    for (int si = 0; si < learning_spp; si++, sampler.start_next_sample()) {
      auto ray = scene.camera.gen_ray((p + sampler.get2d()) / film.size(), sampler.get2d());
      L += radiance(scene, ray, sampler, Vertex::first_vertex()).Lo;
    }
    image0[p] = L / learning_spp;
    if (p.x % 64 == 0)
      set_progress(float(p.x + p.y * film.size().x) / area(film.size()) / 2);
  });

  learning_phase = false;
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = samplers[threadIdx].start_pixel(p, spp);
    auto L = vec3(0.0f);
    for (int si = 0; si < rendering_spp; si++, sampler.start_next_sample()) {
      auto ray = scene.camera.gen_ray((p + sampler.get2d()) / film.size(), sampler.get2d());
      L += radiance(scene, ray, sampler, Vertex::first_vertex()).Lo;
    }
    image1[p] = L / rendering_spp;
    if (p.x % 64 == 0)
      set_progress(0.5f + float(p.x + p.y * film.size().x) / area(film.size()) / 2);
  });

  //   auto image0_estimate = Array2d3f(film.size());
  //   auto image1_estimate = Array2d3f(film.size());
  //   denoise(DenoiseQuality::High, image0_estimate, image0, albedo, normal);
  //   denoise(DenoiseQuality::High, image1_estimate, image1, albedo, normal);
  //   auto variance0 = 0.0, variance1 = 0.0;
  //   parallel_for(film.size(), [&](vec2i p) {
  //     auto Ie0 = image0_estimate[p];
  //     auto Ie1 = image1_estimate[p];
  //     variance0 += average(sqr((image0[p] - Ie0) / max(Ie0, vec3(0.01f))));
  //     variance1 += average(sqr((image1[p] - Ie1) / max(Ie1, vec3(0.01f))));
  //   });
  //   film.pixels = Array2d4f::from(combine(image0, image1, 1.0f / variance0, 1.0f / variance1));
  film.pixels = Array2d4f::from(image1);
  set_progress(1.0f);
}
CachedPathIntegrator::RadianceResult CachedPathIntegrator::radiance(Scene& scene, Ray ray,
                                                                    Sampler& sampler, Vertex pv) {
  auto result = RadianceResult();
  auto wi = -ray.d;
  auto& Lo = result.Lo;

  auto [mit, it] = intersect_tr(ray, sampler);

  if (mit) {
    auto lo = vec3(0.0f);
    if (mit->le)
      lo += *mit->le;
    else if (pv.length + 1 < max_path_length) {
      if (!learning_phase && !pv.is_delta && pv.length >= starting_depth)
        lo += stree.flux_estimate(mit->p, wi, sampler.get3d());
      else {
        if (!pv.inside_subsurface)
          if (auto ls = light_sampler.sample(*mit, sampler.get1d(), sampler.get2d())) {
            if (!hit(Ray(mit->p, ls->wo, 0.0f, ls->distance))) {
              auto tr = transmittance(mit->p, ls->wo, ls->distance, sampler);
              auto f = mit->pg.f(wi, ls->wo);
              if (ls->light->is_delta()) {
                lo += ls->le * tr * f / ls->pdf;
              } else {
                auto mis = balance_heuristic(ls->pdf, mit->pg.pdf(wi, ls->wo));
                lo += ls->le * tr * f / ls->pdf * mis;
              }
            }
          }
        auto ps = mit->pg.sample(wi, sampler.get2d());
        auto nv = Vertex(pv.length + 1, *mit, ps.pdf, false, pv.inside_subsurface);
        auto rr = psl::min(pv.length <= 1 ? 1.0f : ps.f / ps.pdf, 1.0f);
        if (with_probability(rr, sampler)) {
          auto [Li, light_pdf] = radiance(scene, Ray(mit->p, ps.wo), sampler, nv);
          auto mis = light_pdf ? balance_heuristic(ps.pdf, *light_pdf) : 1.0f;
          lo += Li * (ps.f / ps.pdf * mis / rr);
        }
      }
      if (learning_phase)
        stree.add_sample(mit->p, wi, lo);
    }
    auto Tr = transmittance(ray.o, ray.d, mit->t, sampler);
    Lo += lo * Tr * mit->W;
  }

  auto Tr = transmittance(ray.o, ray.d, ray.tmax, sampler);
  if (!it) {
    if (scene.env_light) {
      Lo += Tr * scene.env_light->color(ray.d);
      if (!pv.is_delta)
        result.light_pdf = scene.env_light->pdf(pv.it, ray.d);
    }
    return result;
  }

  if (it->material().is<EmissiveMaterial>()) {
    Lo += Tr * it->material().le({*it, wi});
    if (!pv.is_delta)
      result.light_pdf = light_sampler.pdf(pv.it, *it, ray);
    return result;
  }

  if (pv.length + 1 >= max_path_length)
    return result;

  auto lo = vec3(0.0f);
  if (!learning_phase && !pv.is_delta && pv.length >= starting_depth) {
    lo += stree.flux_estimate(it->p, wi, sampler.get3d());
  } else {
    if (auto rr_m = average(Tr); with_probability(rr_m, sampler)) {
      if (!it->material().is_delta(*it)) {
        if (auto ls = light_sampler.sample(*it, sampler.get1d(), sampler.get2d())) {
          if (!hit(it->spawn_ray(ls->wo, ls->distance))) {
            auto cosine = absdot(ls->wo, it->n);
            auto tr = transmittance(it->p, ls->wo, ls->distance, sampler);
            if (ls->light->is_delta()) {
              auto f = it->material().f({*it, wi, ls->wo, pv.is_delta ? 0.0f : 0.2f});
              lo += ls->le * tr * cosine * f / ls->pdf;
            } else {
              auto [f, bsdf_pdf] =
                  it->material().f_pdf({*it, wi, ls->wo, pv.is_delta ? 0.0f : 0.2f});
              auto mis = balance_heuristic(ls->pdf, bsdf_pdf);
              lo += ls->le * tr * cosine * f / ls->pdf * mis;
            }
          }
        }
      }

      if (auto bs = it->material().sample({*it, wi, sampler.get1d(), sampler.get2d()})) {
        auto cosine = absdot(bs->wo, it->n);
        auto nv =
            Vertex(pv.length + 1, *it, bs->pdf, it->material().is_delta(*it), bs->enter_subsurface);
        auto albedo = luminance(cosine * bs->f / bs->pdf);
        auto rr = psl::min(pv.length <= 1 ? 1.0f : albedo, 1.0f);
        if (with_probability(rr, sampler)) {
          auto [Li, light_pdf] = radiance(scene, it->spawn_ray(bs->wo), sampler, nv);
          auto mis = light_pdf ? balance_heuristic(bs->pdf, *light_pdf) : 1.0f;
          lo += Li * bs->f * (cosine / bs->pdf * mis / rr);
        }
      }
      lo /= rr_m;
    }
  }
  Lo += lo * Tr;
  if (learning_phase)
    stree.add_sample(it->p, wi, lo);

  return result;
}

}  // namespace pine