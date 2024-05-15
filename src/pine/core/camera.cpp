#include <pine/core/sampling.h>
#include <pine/core/camera.h>
#include <pine/core/context.h>

namespace pine {

ThinLenCamera::ThinLenCamera(Film film_, vec3 from, vec3 to, float fov, float len_radius,
                             float focus_distance)
    : position(from),
      c2w(look_at(from, to)),
      film_(MOVE(film_)),
      fov2d(fov * film_.aspect(), fov),
      len_radius(len_radius),
      focus_distance(focus_distance) {
}

static vec2 to_camera_space(vec2 p_film, vec2 fov2d) {
  p_film = (p_film - vec2(0.5f)) * 2;
  return p_film * fov2d;
}

Ray ThinLenCamera::gen_ray(vec2 p_film, vec2 u2) const {
  if (len_radius == 0.0f) {
    auto pc = to_camera_space(p_film, fov2d);
    return Ray(position, normalize(c2w * vec3(pc, 1.0f)));
  } else {
    auto pc = to_camera_space(p_film, fov2d);
    auto dir = normalize(vec3(pc, 1.0f));
    auto p_focus = focus_distance * dir / dir.z;
    auto p_len = vec3(len_radius * sample_disk_polar(u2), 0.0f);
    return Ray(position + p_len, c2w * normalize(p_focus - p_len));
  }
}

void camera_context(Context& ctx) {
  ctx.type<ThinLenCamera>("ThinLenCamera")
      .ctor<Film, vec3, vec3, float>()
      .ctor<Film, vec3, vec3, float, float, float>();
  ctx.type<Camera>("Camera").ctor_variant<ThinLenCamera>().method("film", &Camera::film);
}

}  // namespace pine