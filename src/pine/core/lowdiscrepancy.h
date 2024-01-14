#pragma once

#include <pine/core/vecmath.h>
#include <pine/core/primes.h>
#include <pine/core/rng.h>

#include <pine/psl/vector.h>

namespace pine {

inline float RadicalInverse(int baseIndex, uint64_t a) {
    int base = Primes[baseIndex];
    float invBase = 1.0f / base, invBaseN = 1;
    uint64_t reversedDigits = 0;
    while (a) {
        uint64_t next = a / base;
        uint64_t digits = a - next * base;
        reversedDigits = reversedDigits * base + digits;
        invBaseN *= invBase;
        a = next;
    }

    return psl::min(reversedDigits * invBaseN, OneMinusEpsilon);
}

inline float ScrambledRadicalInverse(int baseIndex, uint64_t a, const uint16_t* perm) {
    int base = Primes[baseIndex];
    float invBase = 1.0f / base, invBaseN = 1;
    uint64_t reversedDigits = 0;
    while (a) {
        uint64_t next = a / base;
        uint64_t digits = a - next * base;
        reversedDigits = reversedDigits * base + perm[digits];
        invBaseN *= invBase;
        a = next;
    }

    float series = perm[0] / (base + 1.0f);
    return psl::min((reversedDigits + series) * invBaseN, OneMinusEpsilon);
}

inline uint64_t InverseRadicalInverse(uint64_t inverse, int base, int nDigits) {
    uint64_t index = 0;
    for (int i = 0; i < nDigits; i++) {
        uint64_t digit = inverse % base;
        inverse /= base;
        index = index * base + digit;
    }

    return index;
}

template <typename T>
void Shuffle(T* samp, int count, int nDimensions, RNG& rng) {
    for (int i = 0; i < count; i++) {
        int other = i + rng.Uniform32u(count - i);
        for (int j = 0; j < nDimensions; j++)
            psl::swap(samp[nDimensions * i + j], samp[nDimensions * other + j]);
    }
}

psl::vector<uint16_t> ComputeRadicalInversePermutations(RNG& rng);

inline uint32_t MultiplyGenerator(const uint32_t* C, uint32_t a) {
    uint32_t v = 0;
    for (int i = 0; a != 0; i++, a >>= 1)
        if (a & 1)
            v ^= C[i];
    return v;
}

inline float SampleGeneratorMatrix(const uint32_t* C, uint32_t a, uint32_t scramble = 0) {
    return psl::min((MultiplyGenerator(C, a) ^ scramble) * 0x1p-32f, OneMinusEpsilon);
}

inline void GrayCodeSample(const uint32_t* C, uint32_t n, uint32_t scramble, float* p) {
    uint32_t v = scramble;
    for (uint32_t i = 0; i < n; i++) {
        p[i] = psl::min(v * 0x1p-32f, OneMinusEpsilon);
        v ^= C[psl::ctz(i + 1)];
    }
}

inline void GrayCodeSample(const uint32_t* C0, const uint32_t* C1, uint32_t n, vec2u32 scramble,
                           vec2* p) {
    uint32_t v[2] = {scramble.x, scramble.y};
    for (uint32_t i = 0; i < n; i++) {
        p[i].x = psl::min(v[0] * 0x1p-32f, OneMinusEpsilon);
        p[i].y = psl::min(v[1] * 0x1p-32f, OneMinusEpsilon);
        v[0] ^= C0[psl::ctz(i + 1)];
        v[1] ^= C1[psl::ctz(i + 1)];
    }
}

inline void VanDerCorput(int nPixelSamples, float* samples, RNG& rng) {
    uint32_t scramble = rng.Uniform32u();
    const uint32_t CVanDerCorput[32] = {
        // clang-format off
        0x80000000, 0x40000000, 0x20000000, 0x10000000, 0x8000000, 0x4000000,
        0x2000000,  0x1000000,  0x800000,   0x400000,   0x200000,  0x100000,
        0x80000,    0x40000,    0x20000,    0x10000,    0x8000,    0x4000,
        0x2000,     0x1000,     0x800,      0x400,      0x200,     0x100,
        0x80,       0x40,       0x20,       0x10,       0x8,       0x4,
        0x2,        0x1
        // clang-format on
    };
    GrayCodeSample(CVanDerCorput, nPixelSamples, scramble, samples);
    Shuffle(samples, nPixelSamples, 1, rng);
}

inline void Sobol2D(int nPixelSamples, vec2* samples, RNG& rng) {
    vec2u32 scramble = {rng.Uniform32u(), rng.Uniform32u()};
    const uint32_t CSobol[2][32] = {
        {0x80000000, 0x40000000, 0x20000000, 0x10000000, 0x8000000, 0x4000000, 0x2000000, 0x1000000,
         0x800000,   0x400000,   0x200000,   0x100000,   0x80000,   0x40000,   0x20000,   0x10000,
         0x8000,     0x4000,     0x2000,     0x1000,     0x800,     0x400,     0x200,     0x100,
         0x80,       0x40,       0x20,       0x10,       0x8,       0x4,       0x2,       0x1},
        {0x80000000, 0xc0000000, 0xa0000000, 0xf0000000, 0x88000000, 0xcc000000, 0xaa000000,
         0xff000000, 0x80800000, 0xc0c00000, 0xa0a00000, 0xf0f00000, 0x88880000, 0xcccc0000,
         0xaaaa0000, 0xffff0000, 0x80008000, 0xc000c000, 0xa000a000, 0xf000f000, 0x88008800,
         0xcc00cc00, 0xaa00aa00, 0xff00ff00, 0x80808080, 0xc0c0c0c0, 0xa0a0a0a0, 0xf0f0f0f0,
         0x88888888, 0xcccccccc, 0xaaaaaaaa, 0xffffffff}};
    GrayCodeSample(CSobol[0], CSobol[1], nPixelSamples, scramble, samples);
    Shuffle(samples, nPixelSamples, 1, rng);
}

}  // namespace pine

