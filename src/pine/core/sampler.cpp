#include <pine/core/sampler.h>
#include <pine/core/context.h>

float bluenoise_1spp(int pixel_i, int pixel_j, int sampleIndex, int sampleDimension);
float bluenoise_2spp(int pixel_i, int pixel_j, int sampleIndex, int sampleDimension);
float bluenoise_4spp(int pixel_i, int pixel_j, int sampleIndex, int sampleDimension);
float bluenoise_8spp(int pixel_i, int pixel_j, int sampleIndex, int sampleDimension);
float bluenoise_16spp(int pixel_i, int pixel_j, int sampleIndex, int sampleDimension);
float bluenoise_32spp(int pixel_i, int pixel_j, int sampleIndex, int sampleDimension);
float bluenoise_64spp(int pixel_i, int pixel_j, int sampleIndex, int sampleDimension);
float bluenoise_128spp(int pixel_i, int pixel_j, int sampleIndex, int sampleDimension);
float bluenoise_256spp(int pixel_i, int pixel_j, int sampleIndex, int sampleDimension);

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

HaltonSampler::HaltonSampler(int spp) : samples_per_pixel(spp) {
  if (samples_per_pixel <= 0)
    SEVERE("`HaltonSampler` should have positive samples per pixel");
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

void HaltonSampler::start_pixel(vec2i p, int sample_index) {
  haltonIndex = 0;
  if (sampleStride > 1) {
    vec2i pm = {psl::mod(p[0], MaxHaltonResolution), psl::mod(p[1], MaxHaltonResolution)};
    for (int i = 0; i < 2; i++) {
      uint64_t dimOffset = inverse_radical_inverse(pm[i], i == 0 ? 2 : 3, baseExponents[i]);
      haltonIndex += dimOffset * baseScales[1 - i] * multInverse[1 - i];
    }
    haltonIndex %= sampleStride;
  }

  haltonIndex += sample_index * sampleStride;
  dimension = 2;
}

void SobolSampler::init(vec2i image_size) {
  auto res = psl::roundup2(max_value(image_size));
  nbase4_digits = psl::log2i(res) + (log2_spp + 1) / 2;
}

uint64_t SobolSampler::compute_sample_index() {
  static const uint8_t permutations[24][4] = {
      {0, 1, 2, 3}, {0, 1, 3, 2}, {0, 2, 1, 3}, {0, 2, 3, 1}, {0, 3, 2, 1}, {0, 3, 1, 2},
      {1, 0, 2, 3}, {1, 0, 3, 2}, {1, 2, 0, 3}, {1, 2, 3, 0}, {1, 3, 2, 0}, {1, 3, 0, 2},
      {2, 1, 0, 3}, {2, 1, 3, 0}, {2, 0, 1, 3}, {2, 0, 3, 1}, {2, 3, 0, 1}, {2, 3, 1, 0},
      {3, 1, 2, 0}, {3, 1, 0, 2}, {3, 2, 1, 0}, {3, 2, 0, 1}, {3, 0, 2, 1}, {3, 0, 1, 2}};

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

BlueSobolSampler::BlueSobolSampler(int spp_) {
  if (spp_ > 256) {
    WARNING("[BlueSobolSampler]Only support up to 256 samples per pixel");
    spp_ = 256;
  }
  samples_per_pixel = psl::roundup2(spp_);
}
float BlueSobolSampler::sample_dimension(int dim) const {
  if (spp() == 1)
    return bluenoise_1spp(pixel.x, pixel.y, index, dim);
  else if (spp() == 2)
    return bluenoise_2spp(pixel.x, pixel.y, index, dim);
  else if (spp() == 4)
    return bluenoise_4spp(pixel.x, pixel.y, index, dim);
  else if (spp() == 8)
    return bluenoise_8spp(pixel.x, pixel.y, index, dim);
  else if (spp() == 16)
    return bluenoise_16spp(pixel.x, pixel.y, index, dim);
  else if (spp() == 32)
    return bluenoise_32spp(pixel.x, pixel.y, index, dim);
  else if (spp() == 64)
    return bluenoise_64spp(pixel.x, pixel.y, index, dim);
  else if (spp() == 128)
    return bluenoise_128spp(pixel.x, pixel.y, index, dim);
  else if (spp() == 256)
    return bluenoise_256spp(pixel.x, pixel.y, index, dim);
  else
    PINE_UNREACHABLE;
}

void MltSampler::ensure_ready(int dim) {
  if (dim >= (int)X.size())
    X.resize(dim + 1);
  auto& Xi = X[dim];

  if (Xi.last_modified_index < last_large_step_index) {
    Xi.value = rng.nextf();
    Xi.last_modified_index = last_large_step_index;
  }

  Xi.backup();
  if (large_step) {
    Xi.value = rng.nextf();
  } else {
    auto n_small = sample_index - Xi.last_modified_index;
    auto normal_sample = psl::Sqrt2 * erf_inv(2 * rng.nextf() - 1);
    auto eff_sigma = sigma * psl::sqrt((float)n_small);
    Xi.value = psl::fract(Xi.value + normal_sample * eff_sigma);
  }
  Xi.last_modified_index = sample_index;
}

void sampler_context(Context& ctx) {
  ctx.type<UniformSampler>("UniformSampler")
      .ctor<int>()
      .method<&UniformSampler::spp>("spp")
      .method<&UniformSampler::start_pixel>("start_pixel")
      .method<&UniformSampler::start_next_sample>("start_next_sample")
      .method<&UniformSampler::get1d>("get1d")
      .method<&UniformSampler::get2d>("get2d");
  ctx.type<HaltonSampler>("HaltonSampler")
      .ctor<int>()
      .method<&HaltonSampler::spp>("spp")
      .method<&HaltonSampler::start_pixel>("start_pixel")
      .method<&HaltonSampler::start_next_sample>("start_next_sample")
      .method<&HaltonSampler::get1d>("get1d")
      .method<&HaltonSampler::get2d>("get2d");
  ctx.type<SobolSampler>("SobolSampler")
      .ctor<int>()
      .method<&SobolSampler::spp>("spp")
      .method<&SobolSampler::start_pixel>("start_pixel")
      .method<&SobolSampler::start_next_sample>("start_next_sample")
      .method<&SobolSampler::get1d>("get1d")
      .method<&SobolSampler::get2d>("get2d");
  ctx.type<BlueSobolSampler>("BlueSampler")
      .ctor<int>()
      .method<&BlueSobolSampler::spp>("spp")
      .method<&BlueSobolSampler::start_pixel>("start_pixel")
      .method<&BlueSobolSampler::start_next_sample>("start_next_sample")
      .method<&BlueSobolSampler::get1d>("get1d")
      .method<&BlueSobolSampler::get2d>("get2d");

  ctx.type<Sampler>("Sampler")
      .ctor_variant<UniformSampler, HaltonSampler, SobolSampler, BlueSobolSampler>();
}

}  // namespace pine