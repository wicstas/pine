#pragma once

#include <pine/core/vecmath.h>

#include <pine/psl/memory.h>

namespace pine {

inline uint64_t Hash64u(uint64_t s) {
  s += 0x9E3779B97f4A7C15ULL;
  s = (s ^ (s >> 30)) * 0xBF58476D1CE4E5B9ULL;
  s = (s ^ (s >> 27)) * 0x94D049BB133111EBULL;
  return s ^ (s >> 31);
}

template <typename T, typename... Ts>
inline uint64_t Hash(T first, Ts... rest) {
  uint64_t bits[(sizeof(T) + 7) / sizeof(uint64_t)] = {};
  psl::memcpy(bits, &first, sizeof(T));

  uint64_t h = 0;
  for (size_t i = 0; i < sizeof(bits) / sizeof(bits[0]); i++)
    h = Hash64u(h ^ bits[i]);

  if constexpr (sizeof...(rest) != 0)
    h = Hash64u(h ^ Hash(rest...));

  return h;
}

template <typename... Ts>
inline float Hashf(Ts... vals) {
  uint64_t h = Hash(vals...);
  uint32_t h32 = h ^ (h >> 32);
  return psl::min(h32 / float(-1u), OneMinusEpsilon);
}

inline uint64_t SplitMix64(uint64_t& s) {
  uint64_t r = s += 0x9E3779B97f4A7C15ULL;
  r = (r ^ (r >> 30)) * 0xBF58476D1CE4E5B9ULL;
  r = (r ^ (r >> 27)) * 0x94D049BB133111EBULL;
  return r ^ (r >> 31);
}

inline uint64_t MixBits(uint64_t v) {
  v ^= (v >> 31);
  v *= 0x7fb5d329728ea185;
  v ^= (v >> 27);
  v *= 0x81dadef4bc2dd44d;
  v ^= (v >> 33);
  return v;
}

inline uint32_t Rotl32(uint32_t x, int k) {
  return (x << k) | (x >> (32 - k));
}
inline uint64_t Rotl64(uint64_t x, int k) {
  return (x << k) | (x >> (64 - k));
}

struct RNG {
  RNG(uint64_t seed = 0) {
    s[0] = SplitMix64(seed);
    s[1] = SplitMix64(seed);
  }

  uint64_t Uniform64u(uint64_t max) {
    return Uniform64u() % max;
  }

  uint32_t Uniform32u(uint32_t max) {
    return Uniform32u() % max;
  }

  uint32_t Uniform32u() {
    uint64_t h = Uniform64u();
    return h ^ (h >> 32);
  }

  uint64_t Uniform64u() {
    const uint64_t s0 = s[0];
    uint64_t s1 = s[1];
    const uint64_t result = s0 + s1;

    s1 ^= s0;
    s[0] = Rotl64(s0, 24) ^ s1 ^ (s1 << 16);
    s[1] = Rotl64(s1, 37);

    return result;
  }

  float uniformf() {
    uint64_t u64 = Uniform64u();
    return psl::min(uint32_t(uint32_t(u64) + uint32_t(u64 >> 32)) * 0x1p-32f, OneMinusEpsilon);
  }
  vec2 uniform2f() {
    return {uniformf(), uniformf()};
  }
  vec3 uniform3f() {
    return {uniformf(), uniformf(), uniformf()};
  }

  uint64_t s[2];
};

}  // namespace pine
