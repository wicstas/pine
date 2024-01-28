#pragma once

#include <pine/core/defines.h>

#include <psl/limits.h>
#include <psl/math.h>

namespace pine {

static constexpr float pi = psl::pi;
static constexpr float pi2 = pi * 2;
static constexpr float pi4 = pi * 4;
static constexpr float epsilon = psl::numeric_limits<float>::epsilon();
static constexpr float one_minus_epsilon = 0x1.fffffep-1;
static constexpr float float_max = psl::numeric_limits<float>::max();
static constexpr float Infinity = psl::numeric_limits<float>::infinity();

inline uint32_t reverse_bits32(uint32_t x) {
  x = (x & 0x55555555) << 1 | (x & 0xaaaaaaaa) >> 1;
  x = (x & 0x33333333) << 2 | (x & 0xcccccccc) >> 2;
  x = (x & 0x0f0f0f0f) << 4 | (x & 0xf0f0f0f0) >> 4;
  x = (x & 0x00ff00ff) << 8 | (x & 0xff00ff00) >> 8;

  return (x << 16) | (x >> 16);
}

inline uint64_t reverse_bits64(uint64_t x) {
  return ((uint64_t)reverse_bits32(x) << 32) | (uint64_t)reverse_bits32(x >> 32);
}

inline uint32_t gray_code(uint32_t n) {
  return (n >> 1) ^ n;
}

inline float erf_inv(float x) {
  x = psl::clamp(x, -.99999f, .99999f);
  auto w = -psl::log((1 - x) * (1 + x));
  float p;
  if (w < 5) {
    w = w - 2.5f;
    p = 2.81022636e-08f;
    p = 3.43273939e-07f + p * w;
    p = -3.5233877e-06f + p * w;
    p = -4.39150654e-06f + p * w;
    p = 0.00021858087f + p * w;
    p = -0.00125372503f + p * w;
    p = -0.00417768164f + p * w;
    p = 0.246640727f + p * w;
    p = 1.50140941f + p * w;
  } else {
    w = psl::sqrt(w) - 3;
    p = -0.000200214257f;
    p = 0.000100950558f + p * w;
    p = 0.00134934322f + p * w;
    p = -0.00367342844f + p * w;
    p = 0.00573950773f + p * w;
    p = -0.0076224613f + p * w;
    p = 0.00943887047f + p * w;
    p = 1.00167406f + p * w;
    p = 2.83297682f + p * w;
  }
  return p * x;
}

void math_context(Context &context);

}  // namespace pine
