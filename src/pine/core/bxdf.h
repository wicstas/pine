#pragma once

#include <pine/core/node.h>

#include <pine/psl/optional.h>
#include <pine/psl/variant.h>

namespace pine {

struct BSDFSample {
  vec3 wo;
  vec3 f;
  float pdf = 0.0f;
};

struct DiffuseBSDF {
  DiffuseBSDF(Node3f albedo) : albedo{psl::move(albedo)} {
  }
  psl::optional<BSDFSample> sample(vec3 wi, float u1, vec2 u, const NodeEvalCtx& nc) const;
  vec3 f(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const;
  float pdf(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const;
  bool is_delta() const {
    return false;
  }

  Node3f albedo;
};

struct ConductorBSDF {
  ConductorBSDF(Node3f albedo, Nodef roughness)
      : albedo{psl::move(albedo)}, roughness{psl::move(roughness)} {
  }
  psl::optional<BSDFSample> sample(vec3 wi, float u1, vec2 u2, const NodeEvalCtx& nc) const;
  vec3 f(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const;
  float pdf(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const;
  bool is_delta() const {
    return false;
  }

  Node3f albedo;
  Nodef roughness;
};

struct DielectricBSDF {
  DielectricBSDF(Node3f albedo, Nodef roughness, Nodef eta)
      : albedo{psl::move(albedo)}, roughness{psl::move(roughness)}, eta{psl::move(eta)} {
  }
  psl::optional<BSDFSample> sample(vec3 wi, float u1, vec2 u2, const NodeEvalCtx& nc) const;
  vec3 f(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const;
  float pdf(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const;
  bool is_delta() const {
    return false;
  }

  Node3f albedo;
  Nodef roughness;
  Nodef eta;
};

struct SpecularReflectionBSDF {
  SpecularReflectionBSDF(Node3f albedo) : albedo{psl::move(albedo)} {
  }
  psl::optional<BSDFSample> sample(vec3 wi, float u1, vec2 u2, const NodeEvalCtx& nc) const;
  vec3 f(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const;
  float pdf(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const;
  bool is_delta() const {
    return true;
  }

  Node3f albedo;
};
struct SpecularRefrectionBSDF {
  SpecularRefrectionBSDF(Node3f albedo, Nodef eta)
      : albedo{psl::move(albedo)}, eta{psl::move(eta)} {
  }
  psl::optional<BSDFSample> sample(vec3 wi, float u1, vec2 u2, const NodeEvalCtx& nc) const;
  vec3 f(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const;
  float pdf(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const;
  bool is_delta() const {
    return true;
  }

  Node3f albedo;
  Nodef eta;
};

struct BSDF : psl::Variant<DiffuseBSDF, ConductorBSDF, DielectricBSDF, SpecularReflectionBSDF,
                           SpecularRefrectionBSDF> {
public:
  using Variant::Variant;

  psl::optional<BSDFSample> sample(vec3 wi, float u1, vec2 u2, const NodeEvalCtx& nc) const {
    return dispatch([&](auto&& x) -> decltype(auto) { return x.sample(wi, u1, u2, nc); });
  }
  vec3 f(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const {
    return dispatch([&](auto&& x) -> decltype(auto) { return x.f(wi, wo, nc); });
  }
  float pdf(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const {
    return dispatch([&](auto&& x) -> decltype(auto) { return x.pdf(wi, wo, nc); });
  }
  bool is_delta() const {
    return dispatch([&](auto&& x) -> decltype(auto) { return x.is_delta(); });
  }
};

}  // namespace pine
