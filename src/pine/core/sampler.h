#pragma once

#include <pine/core/lowdiscrepancy.h>
#include <pine/core/vecmath.h>
#include <pine/core/primes.h>
#include <pine/core/rng.h>

#include <psl/variant.h>
#include <psl/vector.h>

namespace pine {

struct UniformSampler {
  UniformSampler(int samplesPerPixel, int seed = 0) : samplesPerPixel(samplesPerPixel), rng(seed) {
  }

  int SamplesPerPixel() const {
    return samplesPerPixel;
  }
  void StartPixel(const vec2i&, int) {
  }
  void StartNextSample() {
  }
  float Get1D() {
    return rng.uniformf();
  }
  vec2 Get2D() {
    return rng.uniform2f();
  }

  int samplesPerPixel;
  RNG rng;
};

struct StratifiedSampler {
  StratifiedSampler(int xPixelSamples, int yPixelSamples, bool jitter)
      : xPixelSamples(xPixelSamples), yPixelSamples(yPixelSamples), jitter(jitter) {
    samplesPerPixel = xPixelSamples * yPixelSamples;
  }

  int SamplesPerPixel() const {
    return samplesPerPixel;
  }
  void StartPixel(vec2i p, int index) {
    pixel = p;
    sampleIndex = index;
    dimension = 0;
  }
  void StartNextSample() {
    sampleIndex++;
    dimension = 0;
  }
  float Get1D() {
    int stratum = (sampleIndex + hash(pixel, dimension)) % samplesPerPixel;
    dimension += 1;

    float delta = jitter ? rng.uniformf() : 0.5f;

    return (stratum + delta) / SamplesPerPixel();
  }
  vec2 Get2D() {
    int stratum = (sampleIndex + hash(pixel, dimension)) % samplesPerPixel;
    dimension += 2;

    int x = stratum % xPixelSamples, y = stratum / xPixelSamples;
    float dx = jitter ? rng.uniformf() : 0.5f, dy = jitter ? rng.uniformf() : 0.5f;

    return {(x + dx) / xPixelSamples, (y + dy) / yPixelSamples};
  }

  int xPixelSamples, yPixelSamples;
  int samplesPerPixel;
  RNG rng;
  vec2i pixel;
  int sampleIndex;
  int dimension;
  bool jitter;
};

struct HaltonSampler {
  enum class RandomizeStrategy { None, PermuteDigits };

  HaltonSampler(int samplesPerPixel, vec2i filmsize,
                RandomizeStrategy randomizeStrategy = RandomizeStrategy::PermuteDigits);

  int SamplesPerPixel() const {
    return samplesPerPixel;
  }
  void StartPixel(vec2i p, int sampleIndex);
  void StartNextSample() {
    haltonIndex += sampleStride;
    dimension = 2;
  }
  float Get1D() {
    if (dimension >= PrimeTablesize)
      dimension = 2;
    return SampleDimension(dimension++);
  }
  vec2 Get2D() {
    if (dimension + 1 >= PrimeTablesize)
      dimension = 2;
    int dim = dimension;
    dimension += 2;
    return {SampleDimension(dim), SampleDimension(dim + 1)};
  }
  float SampleDimension(int dim) const {
    return ScrambledRadicalInverse(dim, haltonIndex, PermutationForDimension(dim));
  }
  const uint16_t* PermutationForDimension(int dim) const {
    return &radicalInversePermutations[PrimeSums[dim]];
  }

  int samplesPerPixel = 0;
  static constexpr int MaxHaltonResolution = 128;
  vec2i baseScales, baseExponents;
  int multInverse[2] = {};

  int sampleStride = 0;
  int64_t haltonIndex = 0;
  int dimension = 0;
  RandomizeStrategy randomizeStrategy;

  static psl::vector<uint16_t> radicalInversePermutations;
};

struct ZeroTwoSequenceSampler {
  ZeroTwoSequenceSampler(int samplesPerPixel, int nSampledDimensions);

  int SamplesPerPixel() const {
    return samplesPerPixel;
  }
  void StartPixel(vec2i p, int sampleIndex);
  void StartNextSample() {
    currentSampleIndex++;
    current1DDimension = 0;
    current2DDimension = 0;
  }
  float Get1D();
  vec2 Get2D();

  int samplesPerPixel;
  int nSampledDimensions;

  int currentSampleIndex = 0;
  int current1DDimension = 0, current2DDimension = 0;
  psl::vector<psl::vector<float>> samples1D;
  psl::vector<psl::vector<vec2>> samples2D;
  RNG rng;
};

struct MltSampler {
  MltSampler(float sigma, float largeStepProbability, int streamCount, int seed)
      : rng(seed),
        sigma(sigma),
        largeStepProbability(largeStepProbability),
        streamCount(streamCount){};

  int SamplesPerPixel() const {
    return 0;
  }

  void StartPixel(vec2i, int) {
  }

  void StartNextSample() {
    sampleIndex++;
    streamIndex = 0;
    dimension = 0;
    largeStep = rng.uniformf() < largeStepProbability;
  }

  void StartStream(int index) {
    streamIndex = index;
    dimension = 0;
  }

  float Get1D() {
    int dim = GetNextIndex();
    Ensureready(dim);
    return X[dim].value;
  }

  vec2 Get2D() {
    return {Get1D(), Get1D()};
  }

  void Accept() {
    if (largeStep)
      lastLargeStepIndex = sampleIndex;
  }

  void Reject() {
    for (auto& Xi : X)
      if (Xi.lastModificationIndex == sampleIndex)
        Xi.Restore();
    --sampleIndex;
  }

private:
  void Ensureready(int dim);
  int GetNextIndex() {
    return streamIndex + streamCount * dimension++;
  }

  struct PrimarySample {
    void Backup() {
      valueBackup = value;
      modifyBackup = lastModificationIndex;
    }
    void Restore() {
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

struct Sampler : psl::Variant<UniformSampler, StratifiedSampler, HaltonSampler,
                              ZeroTwoSequenceSampler, MltSampler> {
  using Variant::Variant;

  int SamplesPerPixel() const {
    return dispatch([&](auto&& x) { return x.SamplesPerPixel(); });
  }
  void StartPixel(vec2i p, int sampleIndex) {
    return dispatch([&](auto&& x) { return x.StartPixel(p, sampleIndex); });
  }
  void StartNextSample() {
    return dispatch([&](auto&& x) { return x.StartNextSample(); });
  }
  float Get1D() {
    return dispatch([&](auto&& x) { return x.Get1D(); });
  }
  vec2 Get2D() {
    return dispatch([&](auto&& x) { return x.Get2D(); });
  }
  vec3 Get3D() {
    return {Get1D(), Get1D(), Get1D()};
  }
  Sampler Clone() const {
    return dispatch([&](auto&& x) { return Sampler(x); });
  }
};

}  // namespace pine
