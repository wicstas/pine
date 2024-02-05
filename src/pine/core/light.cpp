#include <pine/core/sampling.h>
#include <pine/core/geometry.h>
#include <pine/core/profiler.h>
#include <pine/core/context.h>
#include <pine/core/light.h>
#include <pine/core/color.h>

namespace pine {

LightSample PointLight::sample(vec3 p, vec2) const {
  LightSample ls;
  ls.distance = distance(position, p);
  ls.wo = normalize(position - p, ls.distance);
  ls.pdf = psl::sqr(ls.distance);
  ls.le = color;
  return ls;
}
LightSample DirectionalLight::sample(vec3, vec2) const {
  LightSample ls;
  ls.distance = 1e+10f;
  ls.wo = direction;
  ls.pdf = 1.0f;
  ls.le = color;
  return ls;
}
LightSample AreaLight::sample(vec3 p, vec2 u) const {
  CHECK(geometry);
  LightSample ls;
  auto gs = geometry->sample(p, u);
  ls.wo = gs.w;
  ls.pdf = gs.pdf;
  ls.distance = gs.distance;
  ls.le = geometry->material->le(LeEvalCtx{gs.p, gs.n, gs.uv, -ls.wo});
  return ls;
}

vec3 Sky::color(vec3 wo) const {
  return sun_color * sky_color(wo);
}
LightSample Sky::sample(vec3 n, vec2 u2) const {
  auto ls = LightSample{};
  ls.wo = coordinate_system(n) * uniform_hemisphere(u2);
  ls.distance = float_max;
  ls.pdf = 1 / (2 * Pi);
  ls.le = color(ls.wo);
  return ls;
}
float Sky::pdf(vec3 n, vec3 wo) const {
  if (dot(n, wo) > 0.0f)
    return 1 / (2 * Pi);
  else
    return 0.0f;
}

static vec2i sc2ic(vec2 sc, vec2i image_size) {
  auto ic = vec2i{sc * image_size};
  return clamp(ic, vec2i{0}, image_size - vec2i{1});
}

static vec2 ic2sc(vec2 ic, vec2i image_size) {
  return ic / image_size;
}

Atmosphere::Atmosphere(vec3 sun_direction_, vec3 sun_color, vec2i image_size)
    : sun_direction{normalize(sun_direction_)}, sun_color{sun_color}, image_size{image_size} {
  Profiler _("Atmosphere ctor");
  auto density = Array2d<float>{image_size};
  for_2d(image_size, [&](auto p) {
    auto sc = ic2sc(p, image_size);
    auto wo = uniform_sphere(sc);
    psl::swap(wo.y, wo.z);
    density[p] = length(atmosphere_color(wo, sun_direction, 8, true));
  });
  distr = Distribution2D(density, 15);
}
vec3 Atmosphere::color(vec3 wo) const {
  return sun_color * atmosphere_color(wo, sun_direction, 8);
}
LightSample Atmosphere::sample(vec3, vec2 u2) const {
  auto ls = LightSample{};
  auto ds = distr.sample(u2);
  auto sc = ic2sc(ds.p, image_size);
  ls.wo = uniform_sphere(sc);
  psl::swap(ls.wo.y, ls.wo.z);
  ls.distance = float_max;
  ls.pdf = ds.pdf / (4 * Pi);
  ls.le = sun_color * atmosphere_color(ls.wo, sun_direction, 8, true);
  return ls;
}
float Atmosphere::pdf(vec3, vec3 wo) const {
  psl::swap(wo.y, wo.z);
  auto sc = inverse_uniform_sphere(wo);
  auto ic = sc2ic(sc, image_size);
  return distr.pdf(ic) / (4 * Pi);
}

ImageSky::ImageSky(psl::shared_ptr<Image> image_, vec3 tint)
    : image{psl::move(image_)}, tint{tint} {
  CHECK(image);
  auto density = Array2d<float>{image->size()};
  for_2d(image->size(), [&](auto p) { density[p] = length((*image)[p]); });
  distr = Distribution2D(density, 20);
}

vec3 ImageSky::color(vec3 wo) const {
  psl::swap(wo.y, wo.z);
  auto sc = inverse_uniform_sphere(wo);
  auto ic = sc2ic(sc, image->size());
  return tint * (*image)[ic];
}
LightSample ImageSky::sample(vec3, vec2 u2) const {
  auto ls = LightSample{};
  auto ds = distr.sample(u2);
  auto sc = ic2sc(ds.p, image->size());
  ls.wo = uniform_sphere(sc);
  psl::swap(ls.wo.y, ls.wo.z);
  ls.distance = float_max;
  ls.pdf = ds.pdf / (4 * Pi);
  ls.le = tint * (*image)[ds.p];
  return ls;
}
float ImageSky::pdf(vec3, vec3 wo) const {
  psl::swap(wo.y, wo.z);
  auto sc = inverse_uniform_sphere(wo);
  auto ic = sc2ic(sc, image->size());
  return distr.pdf(ic) / (4 * Pi);
}

void light_context(Context& ctx) {
  ctx.type<PointLight>("PointLight").ctor<vec3, vec3>();
  ctx.type<DirectionalLight>("DirectionalLight").ctor<vec3, vec3>();
  ctx.type<Light>("Light").ctor_variant<PointLight, DirectionalLight>();
  ctx.type<Sky>("Sky").ctor<vec3>();
  ctx.type<Atmosphere>("Atmosphere").ctor<vec3, vec3>();
  ctx.type<ImageSky>("ImageSky")
      .ctor<psl::shared_ptr<Image>>()
      .ctor<psl::shared_ptr<Image>, vec3>();
  ctx.type<EnvironmentLight>("EnvironmentLight").ctor_variant<Atmosphere, Sky, ImageSky>();
}

}  // namespace pine