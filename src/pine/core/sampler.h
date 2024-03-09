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
    return rng.uniformf();
  }
  vec2 get2d() {
    return rng.uniform2f();
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

struct MltSampler {
  MltSampler(float sigma, float largeStepProbability, int streamCount, int seed)
      : rng(seed),
        sigma(sigma),
        largeStepProbability(largeStepProbability),
        streamCount(streamCount) {
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

struct Sampler : private psl::variant<UniformSampler, HaltonSampler, SobolSampler> {
  using variant::variant;

  int spp() const {
    return dispatch([&](auto&& x) { return x.spp(); });
  }
  void init(vec2i image_size) {
    return dispatch([&](auto&& x) { return x.init(image_size); });
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
