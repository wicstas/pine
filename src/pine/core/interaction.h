#pragma once
#include <pine/core/phase_function.h>
#include <pine/core/ray.h>
#include <pine/core/log.h>

#include <psl/variant.h>

namespace pine {

struct SurfaceInteraction {
  Ray spawn_ray(vec3 wo, float distance = float_max) const {
    Ray ray;
    ray.d = wo;
    ray.o = offset_ray_origin(p, face_same_hemisphere(n, ray.d));
    ray.tmin = 1e-8f;
    ray.tmax = distance * (1.0f - 1e-3f);
    return ray;
  }

  void compute_transformation() {
    world_to_local = inverse(coordinate_system(n));
  }
  vec3 to_local(vec3 w) const {
    return world_to_local * w;
  }

  vec3 p;
  vec3 n;
  vec2 uv;
  mat3 world_to_local;

  const Material* material() const;

  const Geometry* geometry = nullptr;
};

struct MediumInteraction {
  MediumInteraction() = default;
  MediumInteraction(vec3 p, float W, PhaseFunction pg) : p(p), W(W), pg(psl::move(pg)) {
  }

  vec3 p;
  float W;
  PhaseFunction pg;
};

struct SpectralMediumInteraction {
  vec3 tr() const {
    return {mits[0] ? 0 : 1, mits[1] ? 0 : 1, mits[2] ? 0 : 1};
  }
  bool is_entire() const {
    return mits[0] && mits[1] && mits[2];
  }

  psl::optional<MediumInteraction>& operator[](int i) {
    return mits[i];
  }
  void iterate(float u, auto f) {
    int indices[3];
    auto pos = 0;
    for (int channel = 0; channel < n_channels; channel++) {
      if (mits[channel])
        indices[pos++] = channel;
    }
    if (pos == 0)
      return;
    auto channel = indices[int(u * pos)];
    auto s = vec3(0.0f);
    s[channel] = 1;
    f(s * pos, *mits[channel]);
  }

  static constexpr int n_channels = 3;
  psl::optional<MediumInteraction> mits[n_channels];
};

struct Interaction : psl::variant<SurfaceInteraction, MediumInteraction> {
  using variant::variant;
  vec3 p() const {
    return dispatch([](auto&& x) { return x.p; });
  }
  vec3 surface_n() const {
    return as<SurfaceInteraction>().n;
  }
};

}  // namespace pine
