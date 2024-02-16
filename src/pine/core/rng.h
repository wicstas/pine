#pragma once

#include <pine/core/vecmath.h>

#include <psl/memory.h>

namespace pine {

inline uint64_t hash64u(uint64_t s) {
  s += 0x9E3779B97f4A7C15ULL;
  s = (s ^ (s >> 30)) * 0xBF58476D1CE4E5B9ULL;
  s = (s ^ (s >> 27)) * 0x94D049BB133111EBULL;
  return s ^ (s >> 31);
}

template <typename T, typename... Ts>
inline uint64_t hash(T first, Ts... rest) {
  uint64_t bits[(sizeof(T) + 7) / sizeof(uint64_t)] = {};
  psl::memcpy(bits, &first, sizeof(T));

  uint64_t h = 0;
  for (size_t i = 0; i < sizeof(bits) / sizeof(bits[0]); i++)
    h = hash64u(h ^ bits[i]);

  if constexpr (sizeof...(rest) != 0)
    h = hash64u(h ^ hash(rest...));

  return h;
}

template <typename... Ts>
inline float hashf(Ts... vals) {
  uint64_t h = hash(vals...);
  uint32_t h32 = h ^ (h >> 32);
  return psl::min(h32 / float(-1u), one_minus_epsilon);
}

inline uint64_t split_mix_64(uint64_t& s) {
  uint64_t r = s += 0x9E3779B97f4A7C15ULL;
  r = (r ^ (r >> 30)) * 0xBF58476D1CE4E5B9ULL;
  r = (r ^ (r >> 27)) * 0x94D049BB133111EBULL;
  return r ^ (r >> 31);
}

inline uint64_t mix_bits(uint64_t v) {
  v ^= (v >> 31);
  v *= 0x7fb5d329728ea185;
  v ^= (v >> 27);
  v *= 0x81dadef4bc2dd44d;
  v ^= (v >> 33);
  return v;
}

inline uint32_t rotl32(uint32_t x, int k) {
  return (x << k) | (x >> (32 - k));
}
inline uint64_t rotl64(uint64_t x, int k) {
  return (x << k) | (x >> (64 - k));
}

struct RNG {
  RNG(uint64_t seed = 0) {
    s[0] = split_mix_64(seed);
    s[1] = split_mix_64(seed);
  }

  uint64_t uniform64u(uint64_t max) {
    return uniform64u() % max;
  }

  uint32_t uniform32u(uint32_t max) {
    return uniform32u() % max;
  }

  uint32_t uniform32u() {
    uint64_t h = uniform64u();
    return h ^ (h >> 32);
  }

  uint64_t uniform64u() {
    const auto s0 = s[0];
    auto s1 = s[1];
    const auto result = s0 + s1;

    s1 ^= s0;
    s[0] = rotl64(s0, 24) ^ s1 ^ (s1 << 16);
    s[1] = rotl64(s1, 37);

    return result;
  }

  float uniformf() {
    auto u64 = uniform64u();
    return psl::min(uint32_t(u64 ^ (u64 >> 32)) * 0x1p-32f, one_minus_epsilon);
  }
  vec2 uniform2f() {
    return {uniformf(), uniformf()};
  }
  vec3 uniform3f() {
    return {uniformf(), uniformf(), uniformf()};
  }

  uint64_t s[2];
};

void rng_context(Context& ctx);

}  // namespace pine
