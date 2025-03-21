int dim = 0;

float radical_inverse(int base_index, uint a) {
  int base = Primes[base_index];
  float invBase = 1.0f / base, invBaseN = 1.0f;
  uint reversedDigits = 0;
  while (a != 0) {
    uint next = a / base;
    uint digits = a - next * base;
    reversedDigits = reversedDigits * base + digits;
    invBaseN *= invBase;
    a = next;
  }

  return min(reversedDigits * invBaseN, OneMinusEpsilon);
}

layout(std430, binding = 10) buffer SSBO10 {
    int sobol_256spp_256d[256 * 256];
};
layout(std430, binding = 11) buffer SSBO11 {
    int scramblingTile[128 * 128 * 8];
};
layout(std430, binding = 12) buffer SSBO12 {
    int rankingTile[128 * 128 * 8];
};
float bluenoise(int pixel_i, int pixel_j, int sampleIndex, int sampleDimension)
{
	// wrap arguments
	pixel_i = pixel_i & 127;
	pixel_j = pixel_j & 127;
	sampleIndex = sampleIndex & 255;
	sampleDimension = sampleDimension & 255;

	// xor index based on optimized ranking
	int rankedSampleIndex = sampleIndex ^ rankingTile[(sampleDimension + (pixel_i + pixel_j*128)*8) % (128 * 128 * 8)];

	// fetch value in sequence
	int value = sobol_256spp_256d[sampleDimension + rankedSampleIndex*256];

	// If the dimension is optimized, xor sequence value based on optimized scrambling
	value = value ^ scramblingTile[(sampleDimension%8) + (pixel_i + pixel_j*128)*8];

	// convert to float and return
	float v = (0.5f+value)/256.0f;
	return v;
}

float next() { return bluenoise(frag_coord.x, frag_coord.y, alpha, dim++);}
vec2 next2() { return vec2(next(), next()); }
vec3 next3() { return vec3(next(), next(), next()); }