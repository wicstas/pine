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
      Fatal("`UniformSampler` should have positive samples per pixel");
  }

  int spp() const {
    return samples_per_pixel;
  }
  void start_pixel(vec2i, int) {
  }
  void start_next_sample() {
  }
  float get1d() {
    return rng.uniformf();
  }
  vec2 get2d() {
    return rng.uniform2f();
  }

private:
  int samples_per_pixel;
  RNG rng;
};

struct StratifiedSampler {
  StratifiedSampler(int xPixelSamples, int yPixelSamples, bool jitter)
      : xPixelSamples(xPixelSamples), yPixelSamples(yPixelSamples), jitter(jitter) {
    if (xPixelSamples <= 0)
      Fatal("`StratifiedSampler` should have positive x-samples per pixel");
    if (yPixelSamples <= 0)
      Fatal("`StratifiedSampler` should have positive y-samples per pixel");
    samples_per_pixel = xPixelSamples * yPixelSamples;
  }

  int spp() const {
    return samples_per_pixel;
  }
  void start_pixel(vec2i p, int index) {
    pixel = p;
    sample_index = index;
    dimension = 0;
  }
  void start_next_sample() {
    sample_index++;
    dimension = 0;
  }
  float get1d() {
    int stratum = (sample_index + hash(pixel, dimension)) % samples_per_pixel;
    dimension += 1;

    float delta = jitter ? rng.uniformf() : 0.5f;

    return (stratum + delta) / spp();
  }
  vec2 get2d() {
    int stratum = (sample_index + hash(pixel, dimension)) % samples_per_pixel;
    dimension += 2;

    int x = stratum % xPixelSamples, y = stratum / xPixelSamples;
    float dx = jitter ? rng.uniformf() : 0.5f, dy = jitter ? rng.uniformf() : 0.5f;

    return {(x + dx) / xPixelSamples, (y + dy) / yPixelSamples};
  }

private:
  int xPixelSamples, yPixelSamples;
  int samples_per_pixel;
  RNG rng;
  vec2i pixel;
  int sample_index;
  int dimension;
  bool jitter;
};

struct HaltonSampler {
  HaltonSampler(int samples_per_pixel);

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

  SobolSampler(int samples_per_pixel, vec2i image_size) : samples_per_pixel(samples_per_pixel) {
    log2_spp = psl::log2i(samples_per_pixel);
    auto res = psl::roundup2(max_value(image_size));
    nbase4_digits = psl::log2i(res) + (log2_spp + 1) / 2;
  }

  int spp() const {
    return samples_per_pixel;
  }
  void start_pixel(vec2i p, int sample_index) {
    dimension = 0;
    sobol_index = (encode_morton64x2(p.x, p.y) << log2_spp) | uint64_t(sample_index);
  }
  void start_next_sample() {
    sobol_index++;
  }
  float get1d() {
    if (dimension >= NSobolDimensions)
      dimension = 0;
    dimension += 1;
    auto u = hash(dimension);
    return sobol_sample(compute_sample_index(), 0, OwenScrambler(u));
  }
  vec2 get2d() {
    if (dimension + 1 >= NSobolDimensions)
      dimension = 0;
    dimension += 2;
    auto u = hash(dimension);
    return {sobol_sample(compute_sample_index(), 0, OwenScrambler(u)),
            sobol_sample(compute_sample_index(), 1, OwenScrambler(u >> 32))};
  }
  uint64_t compute_sample_index() {
    static const uint8_t permutations[24][4] = {
        {0, 1, 2, 3}, {0, 1, 3, 2}, {0, 2, 1, 3}, {0, 2, 3, 1}, {0, 3, 2, 1},
        {0, 3, 1, 2}, {1, 0, 2, 3}, {1, 0, 3, 2}, {1, 2, 0, 3}, {1, 2, 3, 0},
        {1, 3, 2, 0}, {1, 3, 0, 2}, {2, 1, 0, 3}, {2, 1, 3, 0}, {2, 0, 1, 3},
        {2, 0, 3, 1}, {2, 3, 0, 1}, {2, 3, 1, 0}, {3, 1, 2, 0}, {3, 1, 0, 2},
        {3, 2, 1, 0}, {3, 2, 0, 1}, {3, 0, 2, 1}, {3, 0, 1, 2}

    };

    auto si = uint64_t(0);
    // Apply random permutations to full base-4 digits
    auto only_power_of_2 = bool(log2_spp & 1);
    auto last_digit = only_power_of_2 ? 1 : 0;
    for (int i = nbase4_digits - 1; i >= last_digit; --i) {
      int digit_shift = 2 * i - (only_power_of_2 ? 1 : 0);
      int digit = (sobol_index >> digit_shift) & 3;
      uint64_t higher_digits = sobol_index >> (digit_shift + 2);
      int p = (mix_bits(higher_digits ^ (0x55555555u * dimension)) >> 24) % 24;
      digit = permutations[p][digit];
      si |= uint64_t(digit) << digit_shift;
    }

    // Handle power-of-2 (but not 4) sample count
    if (only_power_of_2) {
      int digit = sobol_index & 1;
      si |= digit ^ (mix_bits((sobol_index >> 1) ^ (0x55555555u * dimension)) & 1);
    }

    return si;
  }

private:
  int samples_per_pixel;
  int dimension = 0;
  uint64_t sobol_index = 0;
  int log2_spp;
  int nbase4_digits;
};

struct MltSampler {
  MltSampler(float sigma, float largeStepProbability, int streamCount, int seed)
      : rng(seed),
        sigma(sigma),
        largeStepProbability(largeStepProbability),
        streamCount(streamCount) {
  }

  int spp() const {
    return 0;
  }

  void start_pixel(vec2i, int) {
  }

  void start_next_sample() {
    sample_index++;
    streamIndex = 0;
    dimension = 0;
    largeStep = rng.uniformf() < largeStepProbability;
  }

  void start_stream(int index) {
    streamIndex = index;
    dimension = 0;
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
    if (largeStep)
      lastLargeStepIndex = sample_index;
  }

  void reject() {
    for (auto& Xi : X)
      if (Xi.lastModificationIndex == sample_index)
        Xi.restore();
    --sample_index;
  }

private:
  void ensure_ready(int dim);
  int get_next_index() {
    return streamIndex + streamCount * dimension++;
  }

  struct PrimarySample {
    void backup() {
      valueBackup = value;
      modifyBackup = lastModificationIndex;
    }
    void restore() {
      value = valueBackup;
      lastModificationIndex = modifyBackup;
    }

    float value = 0, valueBackup = 0;
    int64_t lastModificationIndex = 0;
    int64_t modifyBackup = 0;
  };

  RNG rng;
  const float sigma, largeStepProbability;
  psl::vector<PrimarySample> X;
  int64_t sample_index = 0;
  int64_t streamIndex = 0, streamCount = 0;
  int64_t dimension = 0;

  bool largeStep = true;
  int64_t lastLargeStepIndex = 0;
};

struct Sampler : private psl::variant<UniformSampler, StratifiedSampler, HaltonSampler,
                                      SobolSampler, MltSampler> {
  using variant::variant;

  int spp() const {
    return dispatch([&](auto&& x) { return x.spp(); });
  }
  Sampler& start_pixel(vec2i p, int sample_index) {
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
};

void sampler_context(Context& ctx);

}  // namespace pine
