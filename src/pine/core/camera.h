#pragma once

#include <pine/core/film.h>
#include <pine/core/ray.h>

#include <psl/variant.h>

namespace pine {

struct ThinLenCamera {
  ThinLenCamera() = default;
  ThinLenCamera(Film film_, vec3 from, vec3 to, float fov, float len_radius = 0.0f,
                float focus_distance = 1.0f);

  Ray gen_ray(vec2 p_film, vec2 u2) const;
  vec2 project(vec3 p) const;
  Film &film() { return film_; }

  vec3 position;
  mat3 c2w, w2c;
  Film film_;
  vec2 fov2d;
  float len_radius;
  float focus_distance;
};

struct Camera : psl::variant<ThinLenCamera> {
  using variant::variant;

  vec3 position() const {
    return dispatch([&](auto &&x) { return x.position; });
  }
  vec2 fov2d() const {
    return dispatch([&](auto &&x) { return x.fov2d; });
  }
  mat3 c2w() const {
    return dispatch([&](auto &&x) { return x.c2w; });
  }
  mat3 w2c() const {
    return dispatch([&](auto &&x) { return x.w2c; });
  }
  Ray gen_ray(vec2 pfilm, vec2 u2 = vec2(0.5f)) const {
    return dispatch([&](auto &&x) { return x.gen_ray(pfilm, u2); });
  }
  vec2 project(vec3 p) const {
    return dispatch([&](auto &&x) { return x.project(p); });
  }
  Film &film() {
    return dispatch([&](auto &&x) -> Film & { return x.film(); });
  }
};

void camera_context(Context &context);

}  // namespace pine
