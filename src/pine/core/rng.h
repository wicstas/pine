#pragma once

#include <pine/core/vecmath.h>

#include <psl/memory.h>

namespace pine {

inline uint64_t murmur_hash64A(const unsigned char *key, size_t len, uint64_t seed) {
  const uint64_t m = 0xc6a4a7935bd1e995ull;
  const int r = 47;

  uint64_t h = seed ^ (len * m);

  const unsigned char *end = key + 8 * (len / 8);

  while (key != end) {
    uint64_t k;
    psl::memcpy(&k, key, sizeof(uint64_t));
    key += 8;

    k *= m;
    k ^= k >> r;
    k *= m;

    h ^= k;
    h *= m;
  }

  switch (len & 7) {
    case 7: h ^= uint64_t(key[6]) << 48;
    case 6: h ^= uint64_t(key[5]) << 40;
    case 5: h ^= uint64_t(key[4]) << 32;
    case 4: h ^= uint64_t(key[3]) << 24;
    case 3: h ^= uint64_t(key[2]) << 16;
    case 2: h ^= uint64_t(key[1]) << 8;
    case 1: h ^= uint64_t(key[0]); h *= m;
  };

  h ^= h >> r;
  h *= m;
  h ^= h >> r;

  return h;
}

template <typename T>
inline uint64_t hash_buffer(const T *ptr, size_t size, uint64_t seed = 0) {
  return murmur_hash64A((const unsigned char *)ptr, size, seed);
}

template <typename T, typename... Args>
void hash_recursive_copy(char *buf, T v, Args... args) {
  psl::memcpy(buf, &v, sizeof(T));
  if constexpr (sizeof...(Args) > 0)
    hash_recursive_copy(buf + sizeof(T), args...);
}

template <typename... Args>
inline uint64_t hash(Args... args) {
  constexpr size_t sz = (sizeof(Args) + ... + 0);
  uint64_t buf[(sz + 7) / 8];
  hash_recursive_copy((char *)buf, args...);
  return murmur_hash64A((const unsigned char *)buf, sz, 0);
}

template <typename... Ts>
inline float hashf(Ts... vals) {
  uint64_t h = hash(vals...);
  uint32_t h32 = h ^ (h >> 32);
  return psl::min(h32 / float(-1u), one_minus_epsilon);
}

inline uint64_t split_mix_64(uint64_t &s) {
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

  uint64_t next64u(uint64_t max) {
    return next64u() % max;
  }

  uint32_t next32u(uint32_t max) {
    return next32u() % max;
  }

  uint32_t next32u() {
    uint64_t h = next64u();
    return h ^ (h >> 32);
  }

  uint64_t next64u() {
    const auto s0 = s[0];
    auto s1 = s[1];
    const auto result = s0 + s1;

    s1 ^= s0;
    s[0] = rotl64(s0, 24) ^ s1 ^ (s1 << 16);
    s[1] = rotl64(s1, 37);

    return result;
  }

  float nextf() {
    auto u64 = next64u();
    return psl::min(uint32_t(u64 ^ (u64 >> 32)) * 0x1p-32f, one_minus_epsilon);
  }
  vec2 next2f() {
    return {nextf(), nextf()};
  }
  vec3 next3f() {
    return {nextf(), nextf(), nextf()};
  }

  uint64_t s[2];
};

void rng_context(Context &ctx);

}  // namespace pine
