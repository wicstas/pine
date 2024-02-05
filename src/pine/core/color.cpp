#include <pine/core/color.h>
#include <pine/core/geometry.h>

namespace pine {

vec3 uncharted2_filmic(vec3 v) {
  vec3 A = (vec3)0.15f, B = (vec3)0.50f, C = (vec3)0.10f, D = (vec3)0.20f, E = (vec3)0.02f,
       F = (vec3)0.30f;
  auto mapping = [=](vec3 x) {
    return (x * (A * x + C * B) + D * E) / (x * (A * x + B) + D * F) - E / F;
  };
  return mapping(v * 2.0f) * vec3(1.0f) / mapping(vec3(11.2f));
}

vec3 ACES(vec3 v) {
  float a = 2.51f;
  vec3 b = vec3(0.03f);
  float c = 2.43f;
  vec3 d = vec3(0.59f);
  vec3 e = vec3(0.14f);
  v = v * (a * v + b) / (v * (c * v + d) + e);
  return clamp(pow(0.8f * v * (a * v + b) / (v * (c * v + d) + e), 0.8f), vec3(0.0f), vec3(1.0f));
}

vec3 color_map(float v) {
  if (v < 1 / 3.0f) {
    return psl::lerp(v * 3.0f, vec3(0), vec3(0.0f, 0.0f, 0.5f));
  } else if (v < 2 / 3.0f) {
    return psl::lerp((v - 1 / 3.0f) * 3.0f, vec3(0.0f, 0.0f, 0.5f), vec3(0, 1, 0));
  } else if (v < 3 / 3.0f) {
    return psl::lerp((v - 2 / 3.0f) * 3.0f, vec3(0, 1, 0), vec3(1, 0, 0));
  } else {
    return vec3(1, 0, 0);
  }
}
vec3 color_map_auto(float v) {
  v = 1 - 1 / psl::max(v, epsilon);
  return color_map(v);
}

vec3 atmosphere_color(vec3 direction, vec3 sun_dir, int nSamples, bool simulate_real_sun) {
  const vec3 beta_r = vec3(3.8e-6f, 13.5e-6f, 33.1e-6f), beta_m = vec3(21e-6f);
  const float atomsphere_radius = 6420e3f, planet_radius = 6360e3f;
  const float Hr = 1.0f / 7995.0f, Hm = 1.0f / 1200.0f;
  const int nSamplesLight = nSamples / 2;
  const float mu = dot(direction, sun_dir);
  const float phaseR = 3.0f / (16.0f * Pi) * (1.0f + mu * mu), g = 0.76f,
              phaseM = 3.0f / (8.0f * Pi) * (1.0f - g * g) * (1.0f + mu * mu) /
                       ((2.0f + g * g) * psl::pow(1.0f + g * g - 2.0f * g * mu, 1.5f));

  Ray ray = Ray(vec3(0.0f, planet_radius, 0.0f), direction);
  ray.tmax = Sphere::compute_t(ray.o, ray.d, 0.0f, vec3(0.0f), atomsphere_radius);
  float segmentLength = ray.tmax / nSamples, tCurrent = 0.0f;
  float opticalDepthR = 0.0f, opticalDepthM = 0.0f;
  vec3 sumR, sumM;

  for (int i = 0; i < nSamples; i++) {
    vec3 samplePosition = ray(tCurrent + segmentLength * 0.5f);
    float height = length(samplePosition) - planet_radius;
    if (height <= 0)
      break;
    float hr = psl::exp(-height * Hr) * segmentLength, hm = psl::exp(-height * Hm) * segmentLength;
    opticalDepthR += hr;
    opticalDepthM += hm;

    float ltmax = Sphere::compute_t(samplePosition, sun_dir, 0.0f, vec3(0.0f), atomsphere_radius);
    float segmentLengthLight = ltmax / nSamplesLight;
    float opticalDepthRLight = 0.0f, opticalDepthMLight = 0.0f;

    int j = 0;
    float lo2 = dot(samplePosition, samplePosition);
    float ld2 = dot(sun_dir, sun_dir);
    float lod = dot(samplePosition, sun_dir);
    float lt = segmentLengthLight * 0.5f;
    for (; j < nSamplesLight; j++, lt += segmentLengthLight) {
      float heightLight = lo2 + psl::sqr(lt) * ld2 + 2 * lt * lod - planet_radius;
      if (heightLight < 0)
        break;
      opticalDepthRLight += psl::exp(-heightLight * Hr) * segmentLengthLight;
      opticalDepthMLight += psl::exp(-heightLight * Hm) * segmentLengthLight;
    }
    if (j == nSamplesLight) {
      vec3 tau = beta_r * (opticalDepthR + opticalDepthRLight) +
                 beta_m * (opticalDepthM + opticalDepthMLight);
      vec3 tr = exp(-tau);
      sumR += tr * hr;
      sumM += tr * hm;
    }
    tCurrent += segmentLength;
  }

  auto color = sumR * beta_r * phaseR + sumM * beta_m * phaseM;

  auto multiplier = vec3{5.0f};
  if (simulate_real_sun && dot(direction, sun_dir) > 0.998f)
    multiplier *= 1000.0f * vec3{1.0f, 0.9f, 0.8f};
  return color * multiplier;
}

vec3 sky_color(vec3 direction) {
  return psl::sqr(
      psl::lerp(direction.y / 2 + 0.7f, vec3(1.0f, 0.8f, 0.6f), vec3(0.6f, 0.8f, 1.0f)));
}

}  // namespace pine