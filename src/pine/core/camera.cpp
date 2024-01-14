#include <pine/core/sampling.h>
#include <pine/core/camera.h>

namespace pine {

static vec2 to_camera_space(vec2 p_film, vec2 fov2d) {
  p_film = (p_film - vec2(0.5f)) * 2;
  return p_film * fov2d;
}

Ray ThinLenCamera::gen_ray(vec2 p_film, vec2 u2) const {
  auto pc = to_camera_space(p_film, fov2d);
  auto dir = normalize(vec3{pc, 1.0f});
  auto pFocus = focus_distance * dir / dir.z;
  auto pLen = len_radius * vec3{sample_disk_polar(u2), 0.0f};
  auto ray = Ray{vec3{c2w * vec4{pLen, 1.0f}}, mat3{c2w} * normalize(pFocus - pLen)};

  return ray;
}

}  // namespace pine