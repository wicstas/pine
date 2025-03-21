#include <pine/core/sampling.h>
#include <pine/core/geometry.h>
#include <pine/core/profiler.h>
#include <pine/core/context.h>
#include <pine/core/fileio.h>
#include <pine/core/light.h>
#include <pine/core/color.h>

namespace pine {

LightSample PointLight::sample(vec3 p, vec2) const {
  LightSample ls;
  ls.wo = normalize(position - p, ls.distance);
  ls.pdf = psl::sqr(ls.distance);
  ls.le = color;
  return ls;
}
SpotLight::SpotLight(vec3 position, vec3 direction, vec3 color, float falloff_radian,
                     float cutoff_additonal_radian)
    : position(position),
      direction(normalize(direction)),
      color(color),
      falloff_cos(psl::cos(falloff_radian)),
      cutoff_cos(psl::cos(falloff_radian + cutoff_additonal_radian)) {
  if (falloff_radian <= 0.0f)
    SEVERE("`SpotLight` invalid falloff angle");
  if (falloff_radian > Pi2)
    SEVERE("`SpotLight` invalid falloff angle(please use radian, not degree)");
  if (cutoff_additonal_radian < 0.0f)
    SEVERE("`SpotLight` invalid cutoff angle");
  if (falloff_radian + cutoff_additonal_radian > Pi2)
    SEVERE("`SpotLight` invalid cutoff angle(please use radian, not degree)");
};
psl::optional<LightSample> SpotLight::sample(vec3 p, vec2) const {
  LightSample ls;
  ls.wo = normalize(position - p, ls.distance);
  auto cos = -dot(ls.wo, direction);
  if (cos > falloff_cos)
    ls.le = color;
  else if (cos > cutoff_cos)
    ls.le = color * (cos - cutoff_cos) / (falloff_cos - cutoff_cos);
  else
    return psl::nullopt;
  ls.pdf = psl::sqr(ls.distance);
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
psl::optional<LightSample> AreaLight::sample(vec3 p, vec2 u, float u1) const {
  CHECK(geometry);
  LightSample ls;
  if (auto gs = geometry->sample(p, u, u1)) {
    ls.wo = gs->w;
    ls.pdf = gs->pdf;
    ls.distance = gs->distance;
    ls.le = geometry->material->le(LeEvalCtx{gs->p, gs->n, gs->uv, -ls.wo});
    if (ls.le.is_black())
      return psl::nullopt;
  } else {
    return psl::nullopt;
  }
  return ls;
}

vec3 Sky::color(vec3 wo) const {
  return sun_color * sky_color(wo);
}
psl::optional<LightSample> Sky::sample(vec3, vec2 u2) const {
  auto ls = LightSample{};
  ls.wo = uniform_sphere(u2);
  ls.pdf = 1 / (4 * Pi);
  ls.distance = float_max;
  ls.le = color(ls.wo);
  return ls;
}
float Sky::pdf(vec3) const {
  return 1 / (4 * Pi);
}

static vec2 sc2ic(vec2 sc, vec2i image_size) {
  auto ic = sc * image_size;
  return clamp(ic, vec2(0), image_size - vec2(1));
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
psl::optional<LightSample> Atmosphere::sample(vec3, vec2 u2) const {
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
float Atmosphere::pdf(vec3 wo) const {
  psl::swap(wo.y, wo.z);
  auto sc = inverse_uniform_sphere(wo);
  auto ic = sc2ic(sc, image_size);
  return distr.pdf(ic) / (4 * Pi);
}

ImageSky::ImageSky(psl::shared_ptr<Image> image_, vec3 tint, float elevation, float rotation)
    : image{MOVE(image_)}, tint{tint} {
  CHECK(image);
  auto density = Array2d<float>{image->size()};
  for_2d(image->size(), [&](auto p) { density[p] = length((vec3)(*image)[p]); });
  distr = Distribution2D(density, 20);
  if (elevation != 0.0f || rotation != 0.0f) {
    l2w = rotate_x(elevation * Pi) * rotate_y(rotation * Pi * 2);
    w2l = inverse(*l2w);
  }
}
vec3 ImageSky::color(vec3 wo) const {
  if (w2l)
    wo = *w2l * wo;
  psl::swap(wo.y, wo.z);
  auto sc = inverse_uniform_sphere(wo);
  auto ic = sc2ic(sc, image->size());
  return tint * (vec3)image->filtered(ic);
}
psl::optional<LightSample> ImageSky::sample(vec3, vec2 u2) const {
  auto ls = LightSample{};
  auto ds = distr.sample(u2);
  CHECK(!psl::isnan(ds.pdf));
  CHECK(!psl::isinf(ds.pdf));
  CHECK_GE(ds.pdf, 0);
  auto sc = ic2sc(ds.p, image->size());
  ls.wo = uniform_sphere(sc);
  psl::swap(ls.wo.y, ls.wo.z);
  if (l2w)
    ls.wo = *l2w * ls.wo;
  CHECK(!ls.wo.has_inf());
  CHECK(!ls.wo.has_nan());
  ls.distance = float_max;
  ls.pdf = ds.pdf / (4 * Pi);
  ls.le = tint * (vec3)(*image)[ds.p];
  return ls;
}
float ImageSky::pdf(vec3 wo) const {
  if (w2l)
    wo = *w2l * wo;
  psl::swap(wo.y, wo.z);
  auto sc = inverse_uniform_sphere(wo);
  auto ic = sc2ic(sc, image->size());
  return distr.pdf(ic) / (4 * Pi);
}

void light_context(Context& ctx) {
  ctx.type<PointLight>("PointLight").ctor<vec3, vec3>();
  ctx.type<SpotLight>("SpotLight")
      .ctor<vec3, vec3, vec3, float>()
      .ctor<vec3, vec3, vec3, float, float>();
  ctx.type<DirectionalLight>("DirectionalLight").ctor<vec3, vec3>();
  ctx.type<Light>("Light").ctor_variant<PointLight, SpotLight, DirectionalLight>();
  ctx.type<Sky>("Sky").ctor<vec3>();
  ctx.type<Atmosphere>("Atmosphere").ctor<vec3, vec3>();
  ctx.type<ImageSky>("ImageSky")
      .ctor<psl::shared_ptr<Image>>()
      .ctor<psl::shared_ptr<Image>, vec3, float, float>();
  ctx.type<EnvironmentLight>("EnvironmentLight").ctor_variant<Atmosphere, Sky, ImageSky>();
}

}  // namespace pine