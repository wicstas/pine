#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif

#if defined(__GNUC__) || defined(__clang__)
#define PINE_LIKELY(x) __builtin_expect(x, true)
#define PINE_UNLIKELY(x) __builtin_expect(x, false)
#define PINE_RESTRICT __restrict
#define PINE_ALWAYS_INLINE __attribute__((always_inline))
#define PINE_UNREACHABLE                                              \
  {                                                                   \
    Fatal("Should never be_ reached[", __FILE__, ":", __LINE__, "]"); \
    __builtin_unreachable();                                          \
  }
#else
#define PINE_LIKELY(x) x
#define PINE_UNLIKELY(x) x
#define PINE_RESTRICT
#define PINE_ALWAYS_INLINE
#define PINE_UNREACHABLE Fatal("Should never be_ reached");
#endif

namespace pine {

struct SurfaceInteraction;
struct MediumInteraction;
struct PhaseFunction;
struct Mesh;
struct Interaction;
struct Function;
struct Material;
struct Geometry;
struct Context;
struct Sampler;
struct Camera;
struct Scene;
struct Light;
struct Image;
struct Shape;
struct AABB;
struct Ray;
struct RNG;

class Integrator;
struct Accel;

}  // namespace pine
