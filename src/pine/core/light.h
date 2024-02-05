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
  float distance = 1.0f;
  float pdf = 1.0f;
  const Light* light = nullptr;
};

struct LightLeSample {
  vec3 Le;
  Ray ray;
  SpatialPdf pdf;
};

struct PointLight {
  PointLight(vec3 position, vec3 color) : position(position), color(color){};

  LightSample sample(vec3 p, vec2 u2) const;

  vec3 position;
  vec3 color;
};

struct DirectionalLight {
  DirectionalLight(vec3 direction, vec3 color) : direction(normalize(direction)), color(color){};

  LightSample sample(vec3 p, vec2 u2) const;

  vec3 direction;
  vec3 color;
};

struct AreaLight {
  AreaLight(psl::shared_ptr<Geometry> geometry) : geometry{psl::move(geometry)} {};

  LightSample sample(vec3 p, vec2 u2) const;

  psl::shared_ptr<Geometry> geometry;
};

struct Light : psl::variant<PointLight, DirectionalLight, AreaLight> {
  using variant::variant;

  LightSample sample(vec3 p, vec2 u2) const {
    return dispatch([&](auto&& x) { return x.sample(p, u2); });
  }
  bool is_delta() const {
    return !is<AreaLight>();
  }
};

struct Sky {
  Sky(vec3 sun_color) : sun_color{sun_color} {
  }

  vec3 color(vec3 wo) const;
  LightSample sample(vec3 n, vec2 u2) const;
  float pdf(vec3 n, vec3 wo) const;

private:
  vec3 sun_color;
};

struct Atmosphere {
  Atmosphere(vec3 sun_direction, vec3 sun_color, vec2i image_size = {1024, 1024});

  vec3 color(vec3 wo) const;
  LightSample sample(vec3 n, vec2 u2) const;
  float pdf(vec3 n, vec3 wo) const;

private:
  vec3 sun_direction;
  vec3 sun_color;
  vec2i image_size;
  Distribution2D distr;
};

struct ImageSky {
  ImageSky(psl::shared_ptr<Image> image, vec3 tint = vec3{1});

  vec3 color(vec3 wo) const;
  LightSample sample(vec3 n, vec2 u2) const;
  float pdf(vec3 n, vec3 wo) const;

private:
  psl::shared_ptr<Image> image;
  Distribution2D distr;
  vec3 tint;
};

struct EnvironmentLight : psl::variant<Atmosphere, Sky, ImageSky> {
  using variant::variant;

  vec3 color(vec3 wo) const {
    return dispatch([&](auto&& x) { return x.color(wo); });
  }
  psl::optional<LightSample> sample(vec3 n, vec2 u2) const {
    auto s = dispatch([&](auto&& x) { return x.sample(n, u2); });
    if (!(s.pdf > 0))
      return psl::nullopt;
    return s;
  }
  float pdf(vec3 n, vec3 wo) const {
    return dispatch([&](auto&& x) { return x.pdf(n, wo); });
  }
};

void light_context(Context& ctx);

}  // namespace pine