#include <pine/impl/integrator/cachedpath.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/denoise.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

#include <psl/unordered_map.h>
#include <psl/array.h>

namespace pine {

namespace {
struct RadianceSample {
  RadianceSample() = default;
  RadianceSample(vec3 p, vec3 w, vec3 flux) : p(p), w(w), flux(flux) {
  }
  vec3 p;
  vec3 w;
  vec3 flux;
};

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
  SpatialTree(AABB aabb, vec3i64 resolution) : aabb(aabb), grid(resolution) {
    this->aabb.extend_by(1e-4f);
  }
  void add_sample(RadianceSample s) {
    bin_of(s.p, s.w).add_flux(s.flux);
  }
  vec3 flux_estimate(vec3 p, vec3 w) const {
    return bin_of(p, w).flux_estimate();
  }

private:
  DirectionalBin& bin_of(vec3 p, vec3 w) {
    return grid[aabb.relative_position(p) * grid.size()].bin_of(w);
  }
  const DirectionalBin& bin_of(vec3 p, vec3 w) const {
    return const_cast<SpatialTree*>(this)->bin_of(p, w);
  }

  AABB aabb;
  Array3d<SpatialNode> grid;
};

}  // namespace

static SpatialTree stree;
static bool use_estimate = false;

CachedPathIntegrator::CachedPathIntegrator(Accel accel, Sampler sampler, LightSampler light_sampler,
                                           int max_path_length, int max_axis_resolution,
                                           int starting_depth)
    : RTIntegrator{psl::move(accel), psl::move(sampler)},
      light_sampler{psl::move(light_sampler)},
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
void CachedPathIntegrator::render(Scene& scene) {
  RTIntegrator::render(scene);
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("CachedPathIntegrator doesn't support `Plane`, please use `Rect` or `Disk` instead");

  auto aabb = scene.get_aabb();
  auto resolution = vec3i(max_axis_resolution * aabb.diagonal() / max_value(aabb.diagonal()));
  resolution = max(resolution, vec3i(1, 1, 1));
  footprint = max_value(aabb.diagonal()) / max_axis_resolution;
  stree = SpatialTree(aabb, resolution);
  light_sampler.build(&scene);
  auto& film = scene.camera.film();
  auto image0 = Array2d3f(film.size());
  auto image1 = Array2d3f(film.size());
  auto image0_estimate = Array2d3f(film.size());
  auto image1_estimate = Array2d3f(film.size());

  Profiler _("[CachedPath]Render");

  set_progress(0.0f);
  auto primary_spp = psl::max(spp / 2 / primary_ratio, 1);

  auto albedo = Array2d3f(film.size());
  auto normal = Array2d3f(film.size());
  parallel_for(film.size(), [&](vec2i p) {
    auto ray = scene.camera.gen_ray((p + vec2(0.5f)) / film.size(), vec2(0.5f));
    if (auto it = SurfaceInteraction(); intersect(ray, it)) {
      albedo[p] = it.material()->albedo({it.p, it.n, it.uv});
      normal[p] = it.n;
    }
  });

  use_estimate = false;
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = samplers[threadIdx].start_pixel(p, 0);
    auto L = vec3(0.0f);
    for (int i = 0; i < primary_spp; i++) {
      auto ray = scene.camera.gen_ray((p + sampler.get2d()) / film.size(), sampler.get2d());
      L += radiance(scene, ray, sampler, 0, Vertex{}, primary_ratio);
    }
    image0[p] = L / primary_spp;
    if (p.x == 0)
      set_progress(float(p.x + p.y * film.size().x) / area(film.size()) / 2);
  });

  use_estimate = true;
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = samplers[threadIdx].start_pixel(p, 0);
    auto L = vec3(0.0f);
    for (int i = 0; i < primary_spp; i++) {
      auto ray = scene.camera.gen_ray((p + sampler.get2d()) / film.size(), sampler.get2d());
      L += radiance(scene, ray, sampler, 0, Vertex{}, primary_ratio);
    }
    image1[p] = L / primary_spp;
    if (p.x == 0)
      set_progress(0.5f + float(p.x + p.y * film.size().x) / area(film.size()));
  });

  denoise(DenoiseQuality::High, image0_estimate, image0, albedo, normal);
  denoise(DenoiseQuality::High, image1_estimate, image1, albedo, normal);
  auto variance0 = 0.0, variance1 = 0.0;
  parallel_for(film.size(), [&](vec2i p) {
    auto Ie0 = image0_estimate[p];
    auto Ie1 = image1_estimate[p];
    variance0 += average(sqr((image0[p] - Ie0) / max(Ie0, vec3(0.01f))));
    variance1 += average(sqr((image1[p] - Ie1) / max(Ie1, vec3(0.01f))));
  });
  film.pixels = Array2d4f::from(combine(image0, image1, 1.0f / variance0, 1.0f / variance1));
  set_progress(1.0f);
}

vec3 CachedPathIntegrator::radiance(Scene& scene [[maybe_unused]], Ray ray [[maybe_unused]],
                                    Sampler& sampler [[maybe_unused]], int depth [[maybe_unused]],
                                    Vertex v [[maybe_unused]], int ssp [[maybe_unused]]) {
  return {};
  // auto wi = -ray.d;
  // auto it = SurfaceInteraction{};

  // if (!intersect(ray, it)) {
  //   if (scene.env_light) {
  //     auto le = scene.env_light->color(ray.d);
  //     if (v.bsdf_is_delta)
  //       return le;
  //     auto light_pdf = scene.env_light->pdf(v.n, ray.d);
  //     auto mis_term = balance_heuristic(v.sample_pdf, light_pdf);
  //     return le * mis_term;
  //   }
  //   return vec3(0.0f);
  // }

  // if (it.material()->is<EmissiveMaterial>()) {
  //   auto le = it.material()->le({it, wi});
  //   if (v.bsdf_is_delta)
  //     return le;
  //   auto light_pdf = light_sampler.pdf(it.geometry, it, ray, v.n);
  //   auto mis_term = balance_heuristic(v.sample_pdf, light_pdf);
  //   return le * mis_term;
  // }

  // if (depth + 1 == max_path_length)
  //   return vec3(0.0f);

  // if (use_estimate && v.non_delta_path_length >= starting_depth && !it.material()->is_delta())
  //   return stree.flux_estimate(it.p, wi);

  // auto direct_light = [&](const LightSample& ls, float pdf_g) {
  //   if (!hit(it.spawn_ray(ls.wo, ls.distance))) {
  //     auto mec = MaterialEvalCtx(it, -ray.d, ls.wo);
  //     auto f = it.material()->f(mec);
  //     auto cosine = absdot(ls.wo, it.n);
  //     auto mis_term = balance_heuristic(ls.pdf, pdf_g);
  //     return ls.le * cosine * f / ls.pdf * mis_term;
  //   } else {
  //     return vec3(0.0f);
  //   }
  // };

  // auto lo = vec3(0.0f);
  // for (int sp = 0; sp < ssp; sp++) {
  //   if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
  //     auto cosine = absdot(it.n, bs->wo);
  //     auto li = radiance(scene, it.spawn_ray(bs->wo), sampler, depth + 1,
  //                        Vertex{
  //                            .n = it.n,
  //                            .bsdf_is_delta = it.material()->is_delta(),
  //                            .sample_pdf = bs->pdf,
  //                            .non_delta_path_length =
  //                                v.non_delta_path_length + (it.material()->is_delta() ? 0 : 1),
  //                        },
  //                        1);
  //     auto sl = li * cosine * bs->f / bs->pdf;
  //     lo += sl;
  //     if (!use_estimate)
  //       stree.add_sample(RadianceSample(it.p, bs->wo, sl * 2));
  //   }

  //   if (!it.material()->is_delta())
  //     if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
  //       auto sl = direct_light(*ls, it.material()->pdf({it, wi, ls->wo}));
  //       lo += sl;
  //       if (!use_estimate)
  //         stree.add_sample(RadianceSample(it.p, ls->wo, sl * 2));
  //     }

  //   if (depth == 0)
  //     sampler.start_next_sample();
  // }
  // lo /= ssp;

  // return lo;
}

}  // namespace pine