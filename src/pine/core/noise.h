#pragma once
#include <pine/core/vecmath.h>

namespace pine {

float perlin_noise(vec3 np, float frequency = 1.0f, int seed = 0);

vec3 perlin_noise3d(vec3 np, float frequency = 1.0f, int seed = 0);

float turbulence(vec3 np, float frequency, int octaves);

vec3 turbulence3d(vec3 np, float frequency, int octaves);

}  // namespace pine
