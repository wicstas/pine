#include <pine/core/sampler.h>
#include <pine/core/context.h>

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

HaltonSampler::HaltonSampler(int samples_per_pixel) : samples_per_pixel(samples_per_pixel) {
  if (radicalInversePermutations.size() == 0) {
    RNG rng;
    radicalInversePermutations = compute_radical_inverse_permutations(rng);
  }

  for (int i = 0; i < 2; i++) {
    int base = (i == 0) ? 2 : 3;
    int scale = 1, exp = 0;
    while (scale < MaxHaltonResolution) {
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

void HaltonSampler::start_pixel(vec2i p, int sampleIndex) {
  haltonIndex = 0;
  if (sampleStride > 1) {
    vec2i pm = {psl::mod(p[0], MaxHaltonResolution), psl::mod(p[1], MaxHaltonResolution)};
    for (int i = 0; i < 2; i++) {
      uint64_t dimOffset = inverse_radical_inverse(pm[i], i == 0 ? 2 : 3, baseExponents[i]);
      haltonIndex += dimOffset * baseScales[1 - i] * multInverse[1 - i];
    }
    haltonIndex %= sampleStride;
  }

  haltonIndex += sampleIndex * sampleStride;
  dimension = 2;
}

ZeroTwoSequenceSampler::ZeroTwoSequenceSampler(int spp, int nSampledDimensions) {
  samples_per_pixel = psl::roundup2(spp);
  for (int i = 0; i < nSampledDimensions; i++) {
    samples1D.push_back(psl::vector<float>(samples_per_pixel));
    samples2D.push_back(psl::vector<vec2>(samples_per_pixel));
  }
}
void ZeroTwoSequenceSampler::start_pixel(vec2i, int sampleIndex) {
  currentSampleIndex = sampleIndex;
  current1DDimension = current2DDimension = 0;
  if (sampleIndex == 0) {
    for (auto& s : samples1D)
      VanDerCorput(samples_per_pixel, &s[0], rng);
    for (auto& s : samples2D)
      Sobol2D(samples_per_pixel, &s[0], rng);
  }
}
float ZeroTwoSequenceSampler::get1d() {
  if (current1DDimension < (int)samples1D.size())
    return samples1D[current1DDimension++][currentSampleIndex];
  else
    return rng.uniformf();
}
vec2 ZeroTwoSequenceSampler::get2d() {
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

struct OwenScramber {
  OwenScramber(uint32_t seed) : seed(seed) {
  }

  uint32_t operator()(uint32_t v) const {
    if (seed & 1)
      v ^= 1u << 31;
    for (int b = 1; b < 32; b++) {
      uint32_t mask = (~0u) << (32 - b);
      if ((uint32_t)mix_bits((v & mask) ^ seed) & (1u << b))
        v ^= 1u << (31 - b);
    }
    return v;
  }

  uint32_t seed;
};

void MltSampler::ensure_ready(int dim) {
  if (dim >= (int)X.size())
    X.resize(dim + 1);
  PrimarySample& Xi = X[dim];

  if (Xi.lastModificationIndex < lastLargeStepIndex) {
    Xi.value = rng.uniformf();
    Xi.lastModificationIndex = lastLargeStepIndex;
  }

  Xi.backup();
  if (largeStep) {
    Xi.value = rng.uniformf();
  } else {
    int64_t nSmall = sampleIndex - Xi.lastModificationIndex;
    float normalSample = psl::sqrt(2.0f) * erf_inv(2 * rng.uniformf() - 1);
    float effSigma = sigma * psl::sqrt((float)nSmall);
    Xi.value = psl::fract(Xi.value + normalSample * effSigma);
  }
  Xi.lastModificationIndex = sampleIndex;
}

void sampler_context(Context& ctx) {
  ctx.type<UniformSampler>("UniformSampler")
      .ctor<int>()
      .method("spp", &UniformSampler::spp)
      .method("start_pixel", &UniformSampler::start_pixel)
      .method("start_next_sample", &UniformSampler::start_next_sample)
      .method("get1d", &UniformSampler::get1d)
      .method("get2d", &UniformSampler::get2d);
  ctx.type<StratifiedSampler>("StratifiedSampler")
      .ctor<int, int, bool>()
      .method("spp", &StratifiedSampler::spp)
      .method("start_pixel", &StratifiedSampler::start_pixel)
      .method("start_next_sample", &StratifiedSampler::start_next_sample)
      .method("get1d", &StratifiedSampler::get1d)
      .method("get2d", &StratifiedSampler::get2d);
  ctx.type<HaltonSampler>("HaltonSampler")
      .ctor<int>()
      .method("spp", &HaltonSampler::spp)
      .method("start_pixel", &HaltonSampler::start_pixel)
      .method("start_next_sample", &HaltonSampler::start_next_sample)
      .method("get1d", &HaltonSampler::get1d)
      .method("get2d", &HaltonSampler::get2d);
  ctx.type<ZeroTwoSequenceSampler>("ZeroTwoSequenceSampler")
      .ctor<int, int>()
      .method("spp", &ZeroTwoSequenceSampler::spp)
      .method("start_pixel", &ZeroTwoSequenceSampler::start_pixel)
      .method("start_next_sample", &ZeroTwoSequenceSampler::start_next_sample)
      .method("get1d", &ZeroTwoSequenceSampler::get1d)
      .method("get2d", &ZeroTwoSequenceSampler::get2d);

  ctx.type<Sampler>("Sampler")
      .ctor_variant<UniformSampler, StratifiedSampler, HaltonSampler, ZeroTwoSequenceSampler>();

  ctx("radical_inverse") = radical_inverse;
}

}  // namespace pine