#include <pine/core/noise.h>
#include <pine/core/rng.h>

namespace pine {

float perlin_noise(vec3 np, float frequency, int seed) {
  np *= frequency;
  auto uvw = np - floor(np);
  uvw = uvw * uvw * (vec3{3} - 2 * uvw);

  vec3 noise[2][2][2];
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++) {
        auto u = RNG{hash(vec3i((floor(np) + vec3i(x, y, z))), seed)}.uniform2f();
        noise[x][y][z] = spherical_to_cartesian(u.x * Pi * 2, u.y * Pi);
      }
  return 0.5f * (1.0f + perlin_interp(noise, uvw));
}

vec3 perlin_noise3d(vec3 np, float frequency, int seed) {
  return {perlin_noise(np, frequency, seed), perlin_noise(np, frequency, seed + 1),
          perlin_noise(np, frequency, seed + 2)};
}

float turbulence(vec3 np, float frequency, int octaves) {
  float accum = 0.0f;
  float weight = 1.0f;

  for (int i = 0; i < octaves; i++) {
    accum += weight * perlin_noise(np, frequency);
    weight *= 0.5f;
    np *= 2.0f;
  }

  return psl::sqr(accum / (2.0f - weight * 2));
}

vec3 turbulence3d(vec3 np, float frequency, int octaves) {
  vec3 accum;
  float weight = 1.0f;

  for (int i = 0; i < octaves; i++) {
    accum += weight * perlin_noise3d(np, frequency);
    weight *= 0.5f;
    np *= 2.0f;
  }

  return psl::sqr(accum / (2.0f - weight * 2));
}

}  // namespace pine