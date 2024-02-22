#include <pine/impl/integrator/diffuse_filtered_path.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

namespace pine {

namespace {

struct Irradiance {
  vec3 l;
};

struct SpatialNode {
  void add_sample(vec3, vec3, vec3 l) {
    lock.lock();
    alpha += 1;
    auto nalpha = 1.0f / alpha;
    irradiance.l = lerp(nalpha, irradiance.l, l);
    lock.unlock();
  }

  Irradiance irradiance;
  int alpha = 0;
  SpinLock lock;
};

struct SpatialTree {
  SpatialTree() = default;
  SpatialTree(AABB aabb, vec3i resolution) : aabb(aabb), resolution(resolution), nodes(resolution) {
    cube_size = aabb.diagonal() / resolution;
  }
  void add_sample(vec3 p, vec3 n, vec3 li) {
    nodes.at_index(pos_to_index(p)).add_sample(p, n, li);
  }
  size_t pos_to_index(vec3 p) const {
    auto rp = aabb.relative_position(p);
    auto ip = min(vec3i(rp * resolution), resolution - vec3i(1));
    return nodes.index(ip);
  }
  void for_each_sample_near(vec3 p, vec3, float radius, auto f) const {
    // auto r2 = radius * radius;
    auto p0 = resolution * aabb.relative_position(p - vec3(radius));
    auto p1 = resolution * aabb.relative_position(p + vec3(radius));
    auto ip0 = max(vec3i(floor(p0)), vec3i(0));
    auto ip1 = min(vec3i(ceil(p1)), resolution);
    for_3d(ip0, ip1, [&](vec3i ip) {
      // auto x = aabb.lower + ip * cube_size;
      // if (distance_squared(p, x) < r2)
      f(nodes[ip].irradiance);
    });
  };
  void jittered(vec3 p, float radius, vec3 u, auto f) const {
    p = max(p + radius * 2 * (u - vec3(0.5f)), aabb.lower);
    f(nodes.at_index(pos_to_index(p)).irradiance);
  };

private:
  AABB aabb;
  vec3i resolution;
  Array3d<SpatialNode> nodes;
  vec3 cube_size;
};

}  // namespace

static SpatialTree stree;

void DiffuseFilteredPathIntegrator::render(Scene& scene) {
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal(
          "DiffuseFilteredPathIntegrator doesn't support `Plane`, please use `Rect` or `Disk` "
          "instead");
  for (const auto& geometry : scene.geometries)
    if (!geometry->material->is<DiffuseMaterial>())
      Fatal("DiffuseFilteredPathIntegrator only support `DiffuseMaterial`");

  auto aabb = scene.get_aabb();
  int max_axis_resolution = 256;
  auto resolution = vec3i(max_axis_resolution * aabb.diagonal() / max_value(aabb.diagonal()));
  stree = SpatialTree(aabb, resolution);

  accel.build(&scene);
  light_sampler.build(&scene);
  auto& film = scene.camera.film();
  set_progress(0);
  auto total_samples = double(samples_per_pixel + 16) * area(film.size());

  Profiler _("[Integrator]Rendering");
  parallel_for(film.size(), [&](vec2i p) {
    Sampler& sampler = samplers[threadIdx].start_pixel(p, 0);
    auto p_film = vec2(p + sampler.get2d()) / film.size();
    auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
    auto it = Interaction();
    auto is_hit = intersect(ray, it);
    for (int si = 0; si < samples_per_pixel; si++) {
      radiance(scene, ray, it, is_hit, sampler, Vertex::first_vertex());
      sampler.start_next_sample();
      if (p.x == 0)
        set_progress(samples_per_pixel * p.y * film.size().x / total_samples);
    }
  });

  auto current_samples = size_t(samples_per_pixel) * area(film.size());
  auto r0 = 50.0f / film.size()[0] * scene.camera.as<ThinLenCamera>().fov2d[0];
  r0 /= psl::pow(float(samples_per_pixel), 0.25f);

  parallel_for(film.size(), [&](vec2i p) {
    auto p_film = vec2(p + vec2(0.5f)) / film.size();
    auto ray = scene.camera.gen_ray(p_film, vec2(0.5f));
    auto it = Interaction();
    auto Lo = vec3(0);

    if (intersect(ray, it)) {
      auto total_weight = 0.0f;
      auto f = it.material()->as<DiffuseMaterial>().bsdf.albedo({it.p, it.n, it.uv});
      stree.for_each_sample_near(it.p, it.n, r0 * ray.tmax, [&](Irradiance irradiance) {
        total_weight += 1.0f;
        Lo += irradiance.l * f;
      });
      if (total_weight > 0)
        Lo /= total_weight;
    }
    scene.camera.film().add_sample(p, Lo);
    if (p.x == 0)
      set_progress((current_samples + 16 * p.y * film.size().x) / total_samples);
  });

  set_progress(1);
}

DiffuseFilteredPathIntegrator::RadianceResult DiffuseFilteredPathIntegrator::radiance(
    Scene& scene, Ray ray, Interaction it, bool is_hit, Sampler& sampler, Vertex pv) const {
  auto Lo = vec3(0.0f);
  auto wi = -ray.d;

  if (pv.length != 0)
    is_hit = intersect(ray, it);

  if (!is_hit) {
    if (scene.env_light) {
      Lo += scene.env_light->color(ray.d);
      auto light_pdf = scene.env_light->pdf(pv.n, ray.d);
      auto mis_term = balance_heuristic(pv.pdf, light_pdf);
      return {Lo, mis_term};
    }
    return {Lo, psl::nullopt};
  }

  if (it.material()->is<EmissiveMaterial>()) {
    Lo += it.material()->le({it, wi});
    auto light_pdf = light_sampler.pdf(it.geometry, it, ray, pv.n);
    auto mis_term = balance_heuristic(pv.pdf, light_pdf);
    return {Lo, mis_term};
  }

  if (pv.length + 1 >= max_path_length)
    return {Lo, psl::nullopt};

  // TODO: should add sample when failed to sample light?
  if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
    if (!hit(it.spawn_ray(ls->wo, ls->distance))) {
      auto cosine = absdot(ls->wo, it.n);
      auto [f, bsdf_pdf] = it.material()->f_pdf({it, wi, ls->wo});
      auto mis = ls->light->is_delta() ? 1.0f : balance_heuristic(ls->pdf, bsdf_pdf);
      Lo += ls->le * cosine * f / ls->pdf * mis;
      if (pv.length == 0)
        stree.add_sample(it.p, ls->wo,
                         2 * cosine * ls->le / ls->pdf * (ls->light->is_delta() ? 1.0f : mis * 2));
    }
  }

  if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
    auto nv = Vertex(pv.length + 1, it.n, it.p, bs->pdf, it.material()->is_delta());
    auto [Li, mis_direct] = radiance(scene, it.spawn_ray(bs->wo), it, {}, sampler, nv);
    auto cosine = absdot(bs->wo, it.n);
    if (mis_direct) {
      Lo += Li * cosine * bs->f / bs->pdf * (*mis_direct);
      if (pv.length == 0)
        stree.add_sample(it.p, bs->wo, 2 * cosine * Li / bs->pdf * *mis_direct * 2);
    } else {
      Lo += Li * cosine * bs->f / bs->pdf;
      if (pv.length == 0)
        stree.add_sample(it.p, bs->wo, 2 * cosine * Li / bs->pdf);
    }
  }

  return {Lo, psl::nullopt};
}

}  // namespace pine