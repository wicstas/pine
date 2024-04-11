#pragma once
#include <pine/core/vecmath.h>

namespace pine {

float perlin_noise(float p, int seed = 0);
float perlin_noise(vec2 p, int seed = 0);
float perlin_noise(vec3 p, int seed = 0);
vec2 perlin_noise2d(float p, int seed = 0);
vec2 perlin_noise2d(vec2 p, int seed = 0);
vec2 perlin_noise2d(vec3 p, int seed = 0);
vec3 perlin_noise3d(float p, int seed = 0);
vec3 perlin_noise3d(vec2 p, int seed = 0);
vec3 perlin_noise3d(vec3 p, int seed = 0);

float fbm(float p, int octaves);
float fbm(vec2 p, int octaves);
float fbm(vec3 p, int octaves);
vec2 fbm2d(float p, int octaves);
vec2 fbm2d(vec2 p, int octaves);
vec2 fbm2d(vec3 p, int octaves);
vec3 fbm3d(float p, int octaves);
vec3 fbm3d(vec2 p, int octaves);
vec3 fbm3d(vec3 p, int octaves);

}  // namespace pine
