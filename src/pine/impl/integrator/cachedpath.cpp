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

struct SpatialNode {
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

struct SpatialTree {
  SpatialTree() = default;
  SpatialTree(AABB aabb, vec3i64 resolution) : tight_aabb(aabb), aabb(aabb), grid(resolution) {
    this->aabb.extend_by(1e-4f);
  }
  void add_sample(vec3 p, vec3 w, vec3 flux) { bin_of(p, w).add_flux(flux); }
  vec3 flux_estimate(vec3 p, vec3 w, vec3 up) const {
    p += (up - vec3(0.5f)) * tight_aabb.diagonal() / grid.size();
    p = clamp(p, tight_aabb.lower, tight_aabb.upper);
    return bin_of(p, w).flux_estimate();
  }

 private:
  SpatialNode& bin_of(vec3 p, vec3) { return grid[aabb.relative_position(p) * grid.size()]; }
  const SpatialNode& bin_of(vec3 p, vec3 w) const {
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
    : RTIntegrator{MOVE(accel), MOVE(sampler), MOVE(light_sampler)},
      max_path_length{max_path_length},
      max_axis_resolution(max_axis_resolution),
      starting_depth{starting_depth} {
  if (max_path_length <= 0)
    SEVERE("`CachedPathIntegrator` expect `max_path_length` to be positive, get", max_path_length);
  if (max_axis_resolution <= 0)
    SEVERE("`CachedPathIntegrator` expect `max_axis_resolution` to be positive, get",
           max_axis_resolution);
  if (starting_depth < 0)
    SEVERE("`CachedPathIntegrator` expect `starting_depth` to be non-negative, get",
           starting_depth);
}

struct CachedPathIntegrator::Vertex {
  Vertex(int length, int diffuse_length, float pdf, bool is_delta)
      : length(length), diffuse_length(diffuse_length), pdf(pdf), is_delta(is_delta) {}
  Vertex(const Vertex& pv, float pdf, bool is_delta = false)
      : Vertex(pv.length + 1, pv.diffuse_length + (is_delta ? 0 : 1), pdf, is_delta) {}
  static Vertex first_vertex() { return Vertex(0, 0, 0.0f, true); }
  int length;
  int diffuse_length;
  float pdf;
  bool is_delta;
};
void CachedPathIntegrator::render(Scene& scene) {
  RTIntegrator::render(scene);
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      SEVERE("CachedPathIntegrator doesn't support `Plane`, please use `Rect` or `Disk` instead");

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

  auto learning_spp = psl::max(spp / 4, 1);
  auto rendering_spp = psl::max(spp, 1);

  learning_phase = true;
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = samplers[threadIdx].start_pixel(p, 0);
    auto L = vec3(0.0f);
    for (int si = 0; si < learning_spp; si++, sampler.start_next_sample()) {
      auto ray = scene.camera.gen_ray((p + sampler.get2d()) / film.size(), sampler.get2d());
      L += radiance(scene, ray, sampler, Vertex::first_vertex()).Lo;
    }
    image0[p] = L / learning_spp;
    if (p.x % 64 == 0) set_progress(progress_2d(p, film.size()) / 4);
  });

  learning_phase = false;
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = samplers[threadIdx].start_pixel(p, spp);
    auto L = vec3(0.0f);
    for (int si = 0; si < rendering_spp; si++, sampler.start_next_sample()) {
      auto ray = scene.camera.gen_ray((p + sampler.get2d()) / film.size(), sampler.get2d());
      L += radiance(scene, ray, sampler, Vertex::first_vertex()).Lo;
    }
    film.add_sample(p, L / rendering_spp);
    if (p.x % 64 == 0) set_progress(0.25f + progress_2d(p, film.size()) / 4);
  });

  set_progress(1.0f);
}
CachedPathIntegrator::RadianceResult CachedPathIntegrator::radiance(Scene& scene, Ray ray,
                                                                    Sampler& sampler, Vertex pv) {
  auto result = RadianceResult();
  auto wi = -ray.d;
  auto& Lo = result.Lo;

  auto it = intersect(ray);

  auto Tr = transmittance(ray.o, ray.d, ray.tmax, sampler);
  if (!it) {
    if (scene.env_light) {
      Lo += Tr * scene.env_light->color(ray.d);
      if (!pv.is_delta) result.light_pdf = scene.env_light->pdf(ray.d);
    }
    return result;
  }

  if (it->material().is<EmissiveMaterial>()) {
    Lo += Tr * it->material().le({*it, wi});
    if (!pv.is_delta) result.light_pdf = light_sampler.pdf(ray, *it);
    return result;
  }

  if (pv.length + 1 >= max_path_length) return result;

  auto bc = BxdfSampleCtx(*it, wi, 0.0f, pv.diffuse_length > 0);
  auto bxdf = it->material().sample_bxdf(bc, sampler);

  auto beta = vec3(1.0f);
  bxdf.sample_p(beta, bc, sampler);

  auto lo = vec3(0.0f);
  if (!learning_phase && !pv.is_delta && pv.length >= starting_depth) {
    lo += stree.flux_estimate(it->p, wi, sampler.get3d());
  } else {
    if (!bxdf.is_delta()) {
      if (auto ls = light_sampler.sample(it->p, sampler);
          ls && !hit(it->spawn_ray(ls->wo, ls->distance))) {
        auto cosine = absdot(ls->wo, it->n);
        auto tr = transmittance(it->p, ls->wo, ls->distance, sampler);
        auto wo = it->to_local(ls->wo);
        if (ls->light->is_delta()) {
          auto f = bxdf.f(wo);
          lo += ls->le * tr * cosine * f / ls->pdf;
        } else {
          auto f = bxdf.f(wo);
          auto mis = balance_heuristic(ls->pdf, bxdf.pdf(wo));
          lo += ls->le * tr * cosine * f / ls->pdf * mis;
        }
      }
    }
    if (auto bs = bxdf.sample(bc, sampler)) {
      auto cosine = absdot(bs->wo, it->n);
      auto nv = Vertex(pv, bs->pdf, bs->is_delta);
      auto [Li, light_pdf] = radiance(scene, it->spawn_ray(bs->wo), sampler, nv);
      auto mis = light_pdf ? balance_heuristic(bs->pdf, *light_pdf) : 1.0f;
      lo += Li * bs->f * (cosine / bs->pdf * mis);
    }
    stree.add_sample(it->p, wi, lo);
  }
  Lo += Tr * beta * lo;

  return result;
}

}  // namespace pine