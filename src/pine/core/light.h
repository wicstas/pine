#pragma once

#include <pine/core/distribution.h>
#include <pine/core/geometry.h>
#include <pine/core/ray.h>

#include <psl/variant.h>
#include <psl/vector.h>

namespace pine {

struct LightSample {
  vec3 le;
  vec3 wo;
  float distance;
  float pdf;
  bool is_delta;
  const Light* light = nullptr;
};

struct PointLight {
  PointLight(vec3 position, vec3 color) : position(position), color(color){};

  LightSample sample(const Interaction& it, vec2 u2) const;

  vec3 position;
  vec3 color;
};
struct SpotLight {
  SpotLight(vec3 position, vec3 direction, vec3 color, float falloff_radian,
            float cutoff_additonal_radian = 0.0f);

  psl::optional<LightSample> sample(const Interaction& it, vec2 u2) const;

  vec3 position;
  vec3 direction;
  vec3 color;
  float falloff_cos;
  float cutoff_cos;
};
struct DirectionalLight {
  DirectionalLight(vec3 direction, vec3 color) : direction(normalize(direction)), color(color){};

  LightSample sample(const Interaction& it, vec2 u2) const;

  vec3 direction;
  vec3 color;
};
struct AreaLight {
  AreaLight(psl::shared_ptr<Geometry> geometry) : geometry{MOVE(geometry)} {};

  psl::optional<LightSample> sample(const Interaction& it, vec2 u2) const;

  psl::shared_ptr<Geometry> geometry;
};

struct Sky {
  Sky(vec3 sun_color) : sun_color{sun_color} {
  }

  vec3 color(vec3 wo) const;
  psl::optional<LightSample> sample(const Interaction& it, vec2 u2) const;
  float pdf(const Interaction& it, vec3 wo) const;

private:
  vec3 sun_color;
};
struct Atmosphere {
  Atmosphere(vec3 sun_direction, vec3 sun_color, vec2i image_size = {1024, 1024});

  vec3 color(vec3 wo) const;
  psl::optional<LightSample> sample(const Interaction& it, vec2 u2) const;
  float pdf(const Interaction& it, vec3 wo) const;

private:
  vec3 sun_direction;
  vec3 sun_color;
  vec2i image_size;
  Distribution2D distr;
};
struct ImageSky {
  ImageSky(psl::shared_ptr<Image> image, vec3 tint = vec3{1}, float elevation = 0.0f,
           float rotation = 0.0f);

  vec3 color(vec3 wo) const;
  psl::optional<LightSample> sample(const Interaction& it, vec2 u2) const;
  float pdf(const Interaction& it, vec3 wo) const;

private:
  psl::shared_ptr<Image> image;
  Distribution2D distr;
  vec3 tint;
  psl::optional<mat3> l2w, w2l;
};

struct Light
    : psl::variant<PointLight, SpotLight, DirectionalLight, AreaLight, Atmosphere, Sky, ImageSky> {
  using variant::variant;

  psl::optional<LightSample> sample(const Interaction& it, vec2 u2) const {
    auto ls = dispatch([&](auto&& x) -> psl::optional<LightSample> { return x.sample(it, u2); });
    if (ls && ls->pdf > 0)
      return ls;
    else
      return psl::nullopt;
  }
  bool is_delta() const {
    return is<PointLight>() || is<SpotLight>() || is<DirectionalLight>();
  }
};
struct EnvironmentLight : psl::variant<Atmosphere, Sky, ImageSky> {
  using variant::variant;

  vec3 color(vec3 wo) const {
    return dispatch([&](auto&& x) { return x.color(wo); });
  }
  psl::optional<LightSample> sample(const Interaction& it, vec2 u2) const {
    auto ls = dispatch([&](auto&& x) { return x.sample(it, u2); });
    if (ls && ls->pdf > 0)
      return ls;
    else
      return psl::nullopt;
  }
  float pdf(const Interaction& it, vec3 wo) const {
    return dispatch([&](auto&& x) { return x.pdf(it, wo); });
  }
};

void light_context(Context& ctx);

}  // namespace pine