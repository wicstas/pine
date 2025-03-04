#pragma once
#include <pine/core/lowdiscrepancy.h>
#include <pine/core/vecmath.h>
#include <pine/core/primes.h>
#include <pine/core/rng.h>
#include <pine/core/log.h>

#include <psl/variant.h>
#include <psl/vector.h>

namespace pine {

struct UniformSampler {
  UniformSampler(int samples_per_pixel, int seed = 0)
      : samples_per_pixel(samples_per_pixel), rng(seed) {
    if (samples_per_pixel <= 0)
      SEVERE("`UniformSampler` should have positive samples per pixel");
  }

  void init(vec2i) {
  }
  int spp() const {
    return samples_per_pixel;
  }
  void start_pixel(vec2i, int) {
  }
  void start_next_sample() {
  }
  float get1d() {
    return rng.nextf();
  }
  vec2 get2d() {
    return rng.next2f();
  }

private:
  int samples_per_pixel;
  RNG rng;
};

struct HaltonSampler {
  HaltonSampler(int samples_per_pixel);

  void init(vec2i) {
  }
  int spp() const {
    return samples_per_pixel;
  }
  void start_pixel(vec2i p, int sample_index);
  void start_next_sample() {
    haltonIndex += sampleStride;
    dimension = 2;
  }
  float get1d() {
    if (dimension >= PrimeTablesize)
      dimension = 2;
    return sample_dimension(dimension++);
  }
  vec2 get2d() {
    if (dimension + 1 >= PrimeTablesize)
      dimension = 2;
    int dim = dimension;
    dimension += 2;
    return {sample_dimension(dim), sample_dimension(dim + 1)};
  }
  float sample_dimension(int dim) const {
    return scrambled_radical_inverse(dim, haltonIndex, &radicalInversePermutations[PrimeSums[dim]]);
  }

private:
  int samples_per_pixel = 0;
  static constexpr int MaxHaltonResolution = 128;
  vec2i baseScales, baseExponents;
  int multInverse[2] = {};

  int sampleStride = 0;
  int64_t haltonIndex = 0;
  int dimension = 0;

  static psl::vector<uint16_t> radicalInversePermutations;
};

struct SobolSampler {
  struct NoOpRandomizer {
    uint32_t operator()(uint32_t v) const {
      return v;
    }
  };
  struct XorRandomizer {
    uint32_t operator()(uint32_t v) const {
      return v ^ x;
    }
    uint32_t x;
  };
  struct FastOwenScrambler {
    FastOwenScrambler(uint32_t seed) : seed(seed) {
    }
    uint32_t operator()(uint32_t v) const {
      v = reverse_bits32(v);
      v ^= v * 0x3d20adea;
      v += seed;
      v *= (seed >> 16) | 1;
      v ^= v * 0x05526c56;
      v ^= v * 0x53a22864;
      return reverse_bits32(v);
    }

    uint32_t seed;
  };
  struct OwenScrambler {
    OwenScrambler(uint32_t seed) : seed(seed) {
    }
    uint32_t operator()(uint32_t v) const {
      if (seed & 1)
        v ^= 1u << 31;
      for (int b = 1; b < 32; ++b) {
        uint32_t mask = (~0u) << (32 - b);
        if ((uint32_t)mix_bits((v & mask) ^ seed) & (1u << b))
          v ^= 1u << (31 - b);
      }
      return v;
    }

    uint32_t seed;
  };

  SobolSampler(int samples_per_pixel) : samples_per_pixel(samples_per_pixel) {
    log2_spp = psl::log2i(samples_per_pixel);
  }

  void init(vec2i image_size);
  int spp() const {
    return samples_per_pixel;
  }
  void start_pixel(vec2i p, int sample_index) {
    dimension = 0;
    sobol_index = (encode_morton64x2(p.x, p.y) << log2_spp) | uint64_t(sample_index);
  }
  void start_next_sample() {
    dimension = 0;
    sobol_index++;
  }
  float get1d() {
    auto sample_index = compute_sample_index();
    dimension += 1;
    auto u = hash(dimension);
    return sobol_sample(sample_index, 0, FastOwenScrambler(u));
  }
  vec2 get2d() {
    auto sample_index = compute_sample_index();
    dimension += 2;
    auto u = hash(dimension);
    return {sobol_sample(sample_index, 0, FastOwenScrambler(u)),
            sobol_sample(sample_index, 1, FastOwenScrambler(u >> 32))};
  }
  uint64_t compute_sample_index();

private:
  int samples_per_pixel;
  int dimension = 0;
  uint64_t sobol_index = 0;
  int log2_spp;
  int nbase4_digits;
};

struct BlueSobolSampler {
  BlueSobolSampler(int samples_per_pixel);

  void init(vec2i) {
  }
  int spp() const {
    return samples_per_pixel;
  }
  void start_pixel(vec2i p, int sample_index) {
    pixel = p;
    index = sample_index;
  }
  void start_next_sample() {
    dimension = 0;
    index++;
  }
  float get1d() {
    if (dimension >= 256)
      dimension = 2;
    return sample_dimension(dimension++);
  }
  vec2 get2d() {
    if (dimension + 1 >= 256)
      dimension = 2;
    int dim = dimension;
    dimension += 2;
    return {sample_dimension(dim), sample_dimension(dim + 1)};
  }
  float sample_dimension(int dim) const;

private:
  int samples_per_pixel;
  int dimension = 0;
  vec2i pixel;
  int index;
};

struct MltSampler {
  MltSampler(float sigma, float p_large, int seed) : rng(seed), sigma(sigma), p_large(p_large) {
  }

  int spp() const {
    return -1;
  }
  void init(vec2i) {
  }
  Sampler& start_pixel(vec2i, int) {
    PINE_UNREACHABLE;
  }
  void start_next_sample() {
    sample_index++;
    dimension = 0;
    large_step = rng.nextf() < p_large;
  }

  float get1d() {
    int dim = get_next_index();
    ensure_ready(dim);
    return X[dim].value;
  }

  vec2 get2d() {
    return {get1d(), get1d()};
  }

  void accept() {
    if (large_step)
      last_large_step_index = sample_index;
  }

  void reject() {
    for (auto& Xi : X)
      if (Xi.last_modified_index == sample_index)
        Xi.restore();
    --sample_index;
  }

private:
  void ensure_ready(int dim);
  int get_next_index() {
    return dimension++;
  }

  struct PrimarySampleBackup {
    float value = 0;
    int64_t last_modified_index = 0;
  };
  struct PrimarySample : PrimarySampleBackup {
    void backup() {
      backup_ = *this;
    }
    void restore() {
      value = backup_.value;
      last_modified_index = backup_.last_modified_index;
    }

    PrimarySampleBackup backup_;
  };

  RNG rng;
  float sigma, p_large;
  psl::vector<PrimarySample> X;
  int64_t sample_index = 0;
  int64_t dimension = 0;

  bool large_step = true;
  int64_t last_large_step_index = 0;
};

struct Sampler
    : psl::variant<UniformSampler, HaltonSampler, SobolSampler, BlueSobolSampler, MltSampler> {
  using variant::variant;

  int spp() const {
    return dispatch([&](auto&& x) { return x.spp(); });
  }
  void init(vec2i image_size) {
    this->image_size = image_size;
    return dispatch([&](auto&& x) { return x.init(image_size); });
  }
  Sampler& start_pixel(vec2i p, int sample_index) {
    rng = RNG(hash(p, sample_index));
    dispatch([&](auto&& x) { x.start_pixel(p, sample_index); });
    return *this;
  }
  void start_next_sample() {
    return dispatch([&](auto&& x) { return x.start_next_sample(); });
  }
  float get1d() {
    return dispatch([&](auto&& x) { return x.get1d(); });
  }
  vec2 get2d() {
    return dispatch([&](auto&& x) { return x.get2d(); });
  }
  vec3 get3d() {
    return {get1d(), get1d(), get1d()};
  }

  float randf() {
    return rng.nextf();
  }
  vec2 rand2f() {
    return rng.next2f();
  }

private:
  vec2i image_size;
  RNG rng;
};

// Helper function to save one call to `sampler.get1d()` in some cases
inline bool with_probability(float prob, Sampler& sampler) {
  if (prob == 0)
    return false;
  else if (prob == 1)
    return true;
  else
    return sampler.randf() < prob;
}

void sampler_context(Context& ctx);

}  // namespace pine
