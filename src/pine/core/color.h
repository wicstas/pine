#pragma once

#include <pine/core/vecmath.h>

namespace pine {

inline vec3 correct_gamma(vec3 v) {
  return pow(v, 1.0f / 2.2f);
}

inline float luminance(vec3 color) {
  return color.x * 0.212671f + color.y * 0.715160f + color.z * 0.072169f;
}

vec3 uncharted2_filmic(vec3 v);

vec3 ACES(vec3 v);

vec3 color_map(float v);
vec3 color_map_auto(float v);

vec3 atmosphere_color(vec3 direction, vec3 sun_direction, int nsamples, bool simulate_real_sun = false);
vec3 sky_color(vec3 direction);

}  // namespace pine
