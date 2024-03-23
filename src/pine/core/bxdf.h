#pragma once

#include <pine/core/node.h>

#include <psl/optional.h>
#include <psl/variant.h>

namespace pine {

struct BxdfEvalCtx : NodeEvalCtx {
  BxdfEvalCtx(const SurfaceInteraction& it, float min_roughness = 0.0f)
      : NodeEvalCtx(it), min_roughness(min_roughness){};

  float bounded_roughness(float min) const {
    return psl::max(min_roughness, min);
  }

  float min_roughness = 0.0f;
};

struct BSDFSample {
  vec3 wo;
  vec3 f;
  float pdf = 0.0f;
  bool enter_subsurface = false;
};

struct SpecularReflectionBSDF {
  SpecularReflectionBSDF(Node3f albedo) : albedo{psl::move(albedo)} {
  }
  psl::optional<BSDFSample> sample(vec3 wi, float u1, vec2 u2, const BxdfEvalCtx& nc) const;
  vec3 f(vec3 wi, vec3 wo, const BxdfEvalCtx& nc) const;
  float pdf(vec3 wi, vec3 wo, const BxdfEvalCtx& nc) const;
  float roughness_amount(const BxdfEvalCtx&) const {
    return 0.0f;
  }

  Node3f albedo;
};
struct SpecularRefrectionBSDF {
  SpecularRefrectionBSDF(Node3f albedo, Nodef eta)
      : albedo{psl::move(albedo)}, eta{psl::move(eta)} {
  }
  psl::optional<BSDFSample> sample(vec3 wi, float u1, vec2 u2, const BxdfEvalCtx& nc) const;
  vec3 f(vec3 wi, vec3 wo, const BxdfEvalCtx& nc) const;
  float pdf(vec3 wi, vec3 wo, const BxdfEvalCtx& nc) const;
  float roughness_amount(const BxdfEvalCtx&) const {
    return 0.0f;
  }

  Node3f albedo;
  Nodef eta;
};

struct DiffuseBSDF {
  DiffuseBSDF(Node3f albedo) : albedo{psl::move(albedo)} {
  }
  psl::optional<BSDFSample> sample(vec3 wi, float u1, vec2 u, const BxdfEvalCtx& nc) const;
  vec3 f(vec3 wi, vec3 wo, const BxdfEvalCtx& nc) const;
  float pdf(vec3 wi, vec3 wo, const BxdfEvalCtx&) const;
  float roughness_amount(const BxdfEvalCtx&) const {
    return 1.0f;
  }

  Node3f albedo;
};

struct ConductorBSDF {
  ConductorBSDF(Node3f albedo, Nodef roughness)
      : albedo{psl::move(albedo)}, roughness{psl::move(roughness)} {
  }
  psl::optional<BSDFSample> sample(vec3 wi, float u1, vec2 u2, const BxdfEvalCtx& nc) const;
  vec3 f(vec3 wi, vec3 wo, const BxdfEvalCtx& nc) const;
  float pdf(vec3 wi, vec3 wo, const BxdfEvalCtx& nc) const;
  float roughness_amount(const BxdfEvalCtx& nc) const {
    return roughness(nc);
  }

  Node3f albedo;
  Nodef roughness;
};

struct RefractiveDielectricBSDF {
  RefractiveDielectricBSDF(Node3f albedo, Nodef roughness, Nodef eta)
      : albedo{psl::move(albedo)}, roughness{psl::move(roughness)}, eta{psl::move(eta)} {
  }
  psl::optional<BSDFSample> sample(vec3 wi, float u1, vec2 u2, const BxdfEvalCtx& nc) const;
  vec3 f(vec3 wi, vec3 wo, const BxdfEvalCtx& nc) const;
  float pdf(vec3 wi, vec3 wo, const BxdfEvalCtx& nc) const;
  float roughness_amount(const BxdfEvalCtx& nc) const {
    return roughness(nc);
  }

  Node3f albedo;
  Nodef roughness;
  Nodef eta;
};
struct DiffusiveDielectricBSDF {
  DiffusiveDielectricBSDF(Node3f albedo, Nodef roughness, Nodef eta)
      : albedo{psl::move(albedo)}, roughness{psl::move(roughness)}, eta{psl::move(eta)} {
  }
  psl::optional<BSDFSample> sample(vec3 wi, float u1, vec2 u2, const BxdfEvalCtx& nc) const;
  vec3 f(vec3 wi, vec3 wo, const BxdfEvalCtx& nc) const;
  float pdf(vec3 wi, vec3 wo, const BxdfEvalCtx& nc) const;
  float roughness_amount(const BxdfEvalCtx&) const {
    return 1.0f;
  }

  Node3f albedo;
  Nodef roughness;
  Nodef eta;
};

struct BSDF : psl::variant<SpecularReflectionBSDF, SpecularRefrectionBSDF, DiffuseBSDF,
                           ConductorBSDF, RefractiveDielectricBSDF, DiffusiveDielectricBSDF> {
public:
  using variant::variant;

  psl::optional<BSDFSample> sample(vec3 wi, float u1, vec2 u2, const BxdfEvalCtx& nc) const {
    return dispatch([&](auto&& x) { return x.sample(wi, u1, u2, nc); });
  }
  vec3 f(vec3 wi, vec3 wo, const BxdfEvalCtx& nc) const {
    return dispatch([&](auto&& x) { return x.f(wi, wo, nc); });
  }
  float pdf(vec3 wi, vec3 wo, const BxdfEvalCtx& nc) const {
    return dispatch([&](auto&& x) { return x.pdf(wi, wo, nc); });
  }
  float roughness_amount(const BxdfEvalCtx& nc) const {
    return dispatch([&](auto&& x) { return x.roughness_amount(nc); });
  }
};

}  // namespace pine
