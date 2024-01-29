#pragma once

#include <pine/core/film.h>
#include <pine/core/ray.h>

#include <psl/variant.h>

namespace pine {

struct ThinLenCamera {
  ThinLenCamera() = default;
  ThinLenCamera(Film film_, vec3 from, vec3 to, float fov, float len_radius = 0.0f,
                float focus_distance = 1.0f)
      : c2w(look_at(from, to)),
        w2c(inverse(c2w)),
        film_(psl::move(film_)),
        fov2d(fov * film_.aspect(), fov),
        len_radius(len_radius),
        focus_distance(focus_distance) {
  }

  Ray gen_ray(vec2 p_film, vec2 u2) const;
  Film &film() {
    return film_;
  }

  mat4 c2w;
  mat4 w2c;
  Film film_;
  vec2 fov2d;
  float len_radius;
  float focus_distance;
};

struct Camera : psl::Variant<ThinLenCamera> {
  using Variant::Variant;

  Ray gen_ray(vec2 pFilm, vec2 u2) const {
    return dispatch([&](auto &&x) -> decltype(auto) { return x.gen_ray(pFilm, u2); });
  }
  Film &film() {
    return dispatch([&](auto &&x) -> decltype(auto) { return x.film(); });
  }
};

void camera_context(Context &context);

}  // namespace pine
