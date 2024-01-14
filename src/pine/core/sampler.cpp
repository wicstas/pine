#include <pine/core/sampler.h>

namespace pine {

static void extendedGCD(uint64_t a, uint64_t b, int64_t& gcd, int64_t& x, int64_t& y) {
  int d = a / b;
  int r = a - d * b;

  if (r == 0) {
    gcd = b;
    x = 0;
    y = 1;
    return;
  }

  int64_t nx, ny;
  extendedGCD(b, r, gcd, nx, ny);
  x = ny;
  y = nx - d * ny;
}

static uint64_t multiplicativeInverse(int64_t a, int64_t n) {
  int64_t gcd, x, y;
  extendedGCD(a, n, gcd, x, y);
  return psl::mod(x, n);
}

psl::vector<uint16_t> HaltonSampler::radicalInversePermutations;

HaltonSampler::HaltonSampler(int samplesPerPixel, vec2i filmSize,
                             RandomizeStrategy randomizeStrategy)
    : samplesPerPixel(samplesPerPixel), randomizeStrategy(randomizeStrategy) {
  if (radicalInversePermutations.size() == 0) {
    RNG rng;
    radicalInversePermutations = ComputeRadicalInversePermutations(rng);
  }

  for (int i = 0; i < 2; i++) {
    int base = (i == 0) ? 2 : 3;
    int scale = 1, exp = 0;
    while (scale < psl::min(filmSize[i], MaxHaltonResolution)) {
      scale *= base;
      ++exp;
    }
    baseScales[i] = scale;
    baseExponents[i] = exp;
  }

  multInverse[0] = multiplicativeInverse(baseScales[0], baseScales[1]);
  multInverse[1] = multiplicativeInverse(baseScales[1], baseScales[0]);

  sampleStride = baseScales[0] * baseScales[1];
}

void HaltonSampler::StartPixel(vec2i p, int sampleIndex) {
  haltonIndex = 0;
  if (sampleStride > 1) {
    vec2i pm = {psl::mod(p[0], MaxHaltonResolution), psl::mod(p[1], MaxHaltonResolution)};
    for (int i = 0; i < 2; i++) {
      uint64_t dimOffset = InverseRadicalInverse(pm[i], i == 0 ? 2 : 3, baseExponents[i]);
      haltonIndex += dimOffset * baseScales[1 - i] * multInverse[1 - i];
    }
    haltonIndex %= sampleStride;
  }

  haltonIndex += sampleIndex * sampleStride;
  dimension = 2;
}

ZeroTwoSequenceSampler::ZeroTwoSequenceSampler(int spp, int nSampledDimensions)
    : nSampledDimensions(nSampledDimensions) {
  samplesPerPixel = psl::roundup2(spp);
  for (int i = 0; i < nSampledDimensions; i++) {
    samples1D.push_back(psl::vector<float>(samplesPerPixel));
    samples2D.push_back(psl::vector<vec2>(samplesPerPixel));
  }
}
void ZeroTwoSequenceSampler::StartPixel(vec2i, int sampleIndex) {
  currentSampleIndex = sampleIndex;
  current1DDimension = current2DDimension = 0;
  if (sampleIndex == 0) {
    for (auto& s : samples1D)
      VanDerCorput(samplesPerPixel, &s[0], rng);
    for (auto& s : samples2D)
      Sobol2D(samplesPerPixel, &s[0], rng);
  }
}
float ZeroTwoSequenceSampler::Get1D() {
  if (current1DDimension < (int)samples1D.size())
    return samples1D[current1DDimension++][currentSampleIndex];
  else
    return rng.uniformf();
}
vec2 ZeroTwoSequenceSampler::Get2D() {
  if (current2DDimension < (int)samples2D.size())
    return samples2D[current2DDimension++][currentSampleIndex];
  else
    return rng.uniform2f();
}

struct NoRandomizer {
  uint32_t operator()(uint32_t v) const {
    return v;
  }
};

struct BinaryPermuteScrambler {
  BinaryPermuteScrambler(uint32_t permutation) : permutation(permutation) {
  }
  uint32_t operator()(uint32_t v) const {
    return permutation ^ v;
  }
  uint32_t permutation;
};

struct FastOwenScramber {
  FastOwenScramber(uint32_t seed) : seed(seed) {
  }

  uint32_t operator()(uint32_t v) const {
    v = ReverseBits32(v);
    v ^= v * 0x3d20adea;
    v += seed;
    v *= (seed >> 16) | 1;
    v ^= v * 0x05526c56;
    v ^= v * 0x53a22864;
    return ReverseBits32(v);
  }

  uint32_t seed;
};

struct OwenScramber {
  OwenScramber(uint32_t seed) : seed(seed) {
  }

  uint32_t operator()(uint32_t v) const {
    if (seed & 1)
      v ^= 1u << 31;
    for (int b = 1; b < 32; b++) {
      uint32_t mask = (~0u) << (32 - b);
      if ((uint32_t)MixBits((v & mask) ^ seed) & (1u << b))
        v ^= 1u << (31 - b);
    }
    return v;
  }

  uint32_t seed;
};

void MltSampler::EnsureReady(int dim) {
  if (dim >= (int)X.size())
    X.resize(dim + 1);
  PrimarySample& Xi = X[dim];

  if (Xi.lastModificationIndex < lastLargeStepIndex) {
    Xi.value = rng.uniformf();
    Xi.lastModificationIndex = lastLargeStepIndex;
  }

  Xi.Backup();
  if (largeStep) {
    Xi.value = rng.uniformf();
  } else {
    int64_t nSmall = sampleIndex - Xi.lastModificationIndex;
    float normalSample = psl::sqrt(2.0f) * ErfInv(2 * rng.uniformf() - 1);
    float effSigma = sigma * psl::sqrt((float)nSmall);
    Xi.value = psl::fract(Xi.value + normalSample * effSigma);
  }
  Xi.lastModificationIndex = sampleIndex;
}

}  // namespace pine