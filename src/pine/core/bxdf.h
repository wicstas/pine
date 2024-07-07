#pragma once
#include <pine/core/interaction.h>
#include <pine/core/node.h>

#include <psl/optional.h>
#include <psl/variant.h>

namespace pine {

struct BxdfSampleCtx : NodeEvalCtx {
  BxdfSampleCtx(SurfaceInteraction& it, vec3 wi, float min_roughness, bool diffused)
      : NodeEvalCtx(it),
        it(it),
        wi(wi),
        min_roughness(diffused ? min_roughness : 0.0f),
        diffused(diffused) {
  }
  SurfaceInteraction& it;
  vec3 wi;
  float min_roughness;
  bool diffused;
};

struct SubsurfaceSample {
  vec3 wi;
  vec3 p;
  vec3 n;
  vec3 beta = vec3(1, 1, 1);
};

struct BSDFSample {
  vec3 wo;
  vec3 f;
  float pdf = 0.0f;
  bool is_delta = false;
};

struct DiffuseBSDF {
  DiffuseBSDF(vec3 albedo) : albedo(albedo) {
  }
  psl::optional<SubsurfaceSample> sample_p(const BxdfSampleCtx&, Sampler&) const {
    return psl::nullopt;
  }
  psl::optional<BSDFSample> sample(vec3 wi, Sampler& sampler) const;
  vec3 f(vec3 wi, vec3 wo) const;
  float pdf(vec3 wi, vec3 wo) const;
  bool is_delta() const {
    return false;
  }

  vec3 albedo;
};
struct ConductorBSDF {
  ConductorBSDF(vec3 albedo, float roughness) : albedo(albedo), roughness(roughness) {
  }
  psl::optional<SubsurfaceSample> sample_p(const BxdfSampleCtx&, Sampler&) const {
    return psl::nullopt;
  }
  psl::optional<BSDFSample> sample(vec3 wi, Sampler& sampler) const;
  vec3 f(vec3 wi, vec3 wo) const;
  float pdf(vec3 wi, vec3 wo) const;
  bool is_delta() const {
    return roughness < 1e-2f;
  }

  vec3 albedo;
  float roughness;
};
struct RefractiveBSDF {
  RefractiveBSDF(vec3 albedo, float roughness, float ior)
      : albedo(albedo), roughness(roughness), ior(ior) {
  }
  psl::optional<SubsurfaceSample> sample_p(const BxdfSampleCtx&, Sampler&) const {
    return psl::nullopt;
  }
  psl::optional<BSDFSample> sample(vec3 wi, Sampler& sampler) const;
  vec3 f(vec3 wi, vec3 wo) const;
  float pdf(vec3 wi, vec3 wo) const;
  bool is_delta() const {
    return roughness < 1e-2f;
  }

  vec3 albedo;
  float roughness;
  float ior;
};
struct RefractiveDielectricBSDF {
  RefractiveDielectricBSDF(vec3 albedo, float roughness, float ior)
      : albedo(albedo), roughness(roughness), ior(ior) {
  }
  psl::optional<SubsurfaceSample> sample_p(const BxdfSampleCtx&, Sampler&) const {
    return psl::nullopt;
  }
  psl::optional<BSDFSample> sample(vec3 wi, Sampler& sampler) const;
  vec3 f(vec3 wi, vec3 wo) const;
  float pdf(vec3 wi, vec3 wo) const;
  bool is_delta() const {
    return roughness < 1e-2f;
  }

  vec3 albedo;
  float roughness;
  float ior;
};
struct DiffusiveDielectricBSDF {
  DiffusiveDielectricBSDF(vec3 albedo, float roughness, float ior)
      : albedo(albedo), roughness(roughness), ior(ior) {
  }
  psl::optional<SubsurfaceSample> sample_p(const BxdfSampleCtx&, Sampler&) const {
    return psl::nullopt;
  }
  psl::optional<BSDFSample> sample(vec3 wi, Sampler& sampler) const;
  vec3 f(vec3 wi, vec3 wo) const;
  float pdf(vec3 wi, vec3 wo) const;
  bool is_delta() const {
    return false;
  }

  vec3 albedo;
  float roughness;
  float ior;
};

struct BSSRDF {
  BSSRDF(vec3 albedo, float ior, vec3 sigma_s) : albedo(albedo), ior(ior), sigma_s{sigma_s} {
  }
  psl::optional<SubsurfaceSample> sample_p(const BxdfSampleCtx& bc, Sampler& sampler) const;
  psl::optional<BSDFSample> sample(vec3 wi, Sampler& sampler) const;
  vec3 f(vec3 wi, vec3 wo) const;
  float pdf(vec3 wi, vec3 wo) const;
  bool is_delta() const {
    return false;
  }

  vec3 albedo;
  float ior;
  vec3 sigma_s;
};

struct BXDF : psl::variant<DiffuseBSDF, ConductorBSDF, RefractiveBSDF, RefractiveDielectricBSDF,
                           DiffusiveDielectricBSDF, BSSRDF> {
  using variant::variant;
  void sample_p(vec3& beta, const BxdfSampleCtx& bc, Sampler& sampler);
  psl::optional<BSDFSample> sample(const BxdfSampleCtx& bc, Sampler& sampler) const {
    return fmap(dispatch([&](auto&& x) { return x.sample(wi, sampler); }),
                [&](auto& bs) { bs.wo = bc.it.to_world(bs.wo); });
  }
  vec3 f(vec3 wo) const {
    return dispatch([&](auto&& x) { return x.f(wi, wo); });
  }
  float pdf(vec3 wo) const {
    return dispatch([&](auto&& x) { return x.pdf(wi, wo); });
  }
  bool is_delta() const {
    return dispatch([&](auto&& x) { return x.is_delta(); });
  }

  vec3 wi;
};

}  // namespace pine
