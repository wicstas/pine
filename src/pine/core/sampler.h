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
    sampleIndex = index;
    dimension = 0;
  }
  void start_next_sample() {
    sampleIndex++;
    dimension = 0;
  }
  float get1d() {
    int stratum = (sampleIndex + hash(pixel, dimension)) % samples_per_pixel;
    dimension += 1;

    float delta = jitter ? rng.uniformf() : 0.5f;

    return (stratum + delta) / spp();
  }
  vec2 get2d() {
    int stratum = (sampleIndex + hash(pixel, dimension)) % samples_per_pixel;
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
  int sampleIndex;
  int dimension;
  bool jitter;
};

struct HaltonSampler {
  HaltonSampler(int samples_per_pixel);

  int spp() const {
    return samples_per_pixel;
  }
  void start_pixel(vec2i p, int sampleIndex);
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
    sampleIndex++;
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
      lastLargeStepIndex = sampleIndex;
  }

  void reject() {
    for (auto& Xi : X)
      if (Xi.lastModificationIndex == sampleIndex)
        Xi.restore();
    --sampleIndex;
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
  int64_t sampleIndex = 0;
  int64_t streamIndex = 0, streamCount = 0;
  int64_t dimension = 0;

  bool largeStep = true;
  int64_t lastLargeStepIndex = 0;
};

struct Sampler
    : private psl::variant<UniformSampler, StratifiedSampler, HaltonSampler, MltSampler> {
  using variant::variant;

  int spp() const {
    return dispatch([&](auto&& x) { return x.spp(); });
  }
  Sampler& start_pixel(vec2i p, int sampleIndex) {
    dispatch([&](auto&& x) { x.start_pixel(p, sampleIndex); });
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
