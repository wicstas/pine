#include <pine/core/scattering.h>
#include <pine/core/bxdf.h>

namespace pine {

psl::optional<BSDFSample> DiffuseBSDF::sample(vec3 wi, float, vec2 u, const NodeEvalCtx& nc) const {
  BSDFSample bs;

  vec3 wo = cosine_weighted_hemisphere(u);
  if (CosTheta(wi) < 0)
    wo = -wo;
  DCHECK(SameHemisphere(wi, wo));

  bs.wo = wo;
  bs.pdf = AbsCosTheta(bs.wo) / pi;
  bs.f = albedo(nc) / pi;
  if (bs.pdf == 0.0f)
    return psl::nullopt;
  return bs;
}

vec3 DiffuseBSDF::f(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const {
  if (!SameHemisphere(wi, wo))
    return vec3(0.0f);
  return albedo(nc) / pi;
}
float DiffuseBSDF::pdf(vec3 wi, vec3 wo, const NodeEvalCtx&) const {
  if (!SameHemisphere(wi, wo))
    return Epsilon;
  return AbsCosTheta(wo) / pi;
}

psl::optional<BSDFSample> ConductorBSDF::sample(vec3 wi, float, vec2 u2,
                                                const NodeEvalCtx& nc) const {
  BSDFSample bs;

  float alpha = psl::clamp(psl::sqr(roughness(nc)), 0.001f, 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);
  vec3 wm = distrib.SampleWm(wi, u2);

  vec3 wo = Reflect(wi, wm);
  if (!SameHemisphere(wi, wo))
    return psl::nullopt;

  vec3 fr = FrSchlick(albedo(nc), AbsCosTheta(wm));

  bs.wo = wo;
  bs.pdf = distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
  bs.f = fr * distrib.D(wm) * distrib.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo));

  return bs;
}

vec3 ConductorBSDF::f(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const {
  if (!SameHemisphere(wi, wo))
    return {};

  float alpha = psl::clamp(psl::sqr(roughness(nc)), 0.001f, 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);

  vec3 wh = normalize(wi + wo);

  vec3 fr = FrSchlick(albedo(nc), AbsCosTheta(wh));

  return fr * distrib.D(wh) * distrib.G(wo, wi) / (4 * AbsCosTheta(wo) * AbsCosTheta(wi));
}
float ConductorBSDF::pdf(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const {
  if (!SameHemisphere(wi, wo))
    return {};

  float alpha = psl::clamp(psl::sqr(roughness(nc)), 0.001f, 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);

  vec3 wh = normalize(wi + wo);

  return distrib.pdf(wi, wh) / (4 * absdot(wi, wh));
}

psl::optional<BSDFSample> DielectricBSDF::sample(vec3 wi, float u1, vec2 u2,
                                                 const NodeEvalCtx& nc) const {
  BSDFSample bs;
  float etap = eta(nc);
  if (CosTheta(wi) < 0)
    etap = 1.0f / etap;
  float fr = FrDielectric(AbsCosTheta(wi), etap);

  float alpha = psl::clamp(psl::sqr(roughness(nc)), 0.001f, 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);
  vec3 wm = distrib.SampleWm(wi, u2);

  if (u1 < fr) {
    vec3 wo = Reflect(wi, wm);
    if (!SameHemisphere(wi, wo))
      return psl::nullopt;

    bs.wo = wo;
    bs.pdf = fr * distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
    bs.f = albedo(nc) * fr * distrib.D(wm) * distrib.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo));
  } else {
    vec3 wo;
    if (!Refract(wi, wm, etap, wo))
      return psl::nullopt;
    float cosThetaO = CosTheta(wo), cosThetaI = CosTheta(wi);
    bs.wo = wo;
    float denom = psl::sqr(dot(wo, wm) + dot(wi, wm) / etap);
    float dwm_dwo = absdot(wo, wm) / denom;
    bs.pdf = (1.0f - fr) * distrib.pdf(wi, wm) * dwm_dwo;
    bs.f = albedo(nc) * (1.0f - fr) * distrib.D(wm) * distrib.G(wi, wo) *
           psl::abs(dot(wo, wm) * dot(wi, wm) / denom / cosThetaI / cosThetaO);
  }
  return bs;
}

vec3 DielectricBSDF::f(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const {
  float alpha = psl::clamp(psl::sqr(roughness(nc)), 0.001f, 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);

  float cosThetaO = CosTheta(wo), cosThetaI = CosTheta(wi);
  bool reflect = cosThetaI * cosThetaO > 0;
  float etap = eta(nc);
  if (!reflect)
    etap = 1.0f / etap;

  vec3 wm = FaceForward(normalize(wi * etap + wo), vec3(0, 0, 1));
  if (dot(wm, wo) * cosThetaI < 0.0f || dot(wm, wi) * cosThetaO < 0.0f)
    return {};

  float fr = FrDielectric(AbsCosTheta(wi), eta(nc));

  if (reflect) {
    return albedo(nc) * fr * distrib.D(wm) * distrib.G(wo, wi) / (4 * cosThetaI * cosThetaO);
  } else {
    float denom = psl::sqr(dot(wo, wm) + dot(wi, wm) / etap) * cosThetaI * cosThetaO;
    return albedo(nc) * (1.0f - fr) * distrib.D(wm) * distrib.G(wi, wo) *
           psl::abs(dot(wo, wm) * dot(wi, wm) / denom);
  }
}
float DielectricBSDF::pdf(vec3 wi, vec3 wo, const NodeEvalCtx& nc) const {
  float alpha = psl::clamp(psl::sqr(roughness(nc)), 0.001f, 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);

  float cosThetaO = CosTheta(wo), cosThetaI = CosTheta(wi);
  bool reflect = cosThetaI * cosThetaO > 0;
  float etap = eta(nc);
  if (!reflect)
    etap = 1.0f / etap;

  vec3 wm = FaceForward(normalize(wi * etap + wo), vec3(0, 0, 1));
  if (dot(wm, wo) * cosThetaI < 0.0f || dot(wm, wi) * cosThetaO < 0.0f)
    return {};

  float fr = FrDielectric(AbsCosTheta(wi), eta(nc));

  if (reflect) {
    return fr * distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
  } else {
    float denom = psl::sqr(dot(wo, wm) + dot(wi, wm) / etap);
    float dwm_dwo = absdot(wo, wm) / denom;
    return (1.0f - fr) * distrib.pdf(wi, wm) * dwm_dwo;
  }
}

psl::optional<BSDFSample> SpecularReflectionBSDF::sample(vec3 wi, float, vec2,
                                                         const NodeEvalCtx& nc) const {
  auto bs = BSDFSample{};
  bs.f = albedo(nc);
  bs.wo = Reflect(wi, vec3{0, 0, 1});
  bs.pdf = psl::max(AbsCosTheta(bs.wo), Epsilon);
  return bs;
}

vec3 SpecularReflectionBSDF::f(vec3, vec3, const NodeEvalCtx&) const {
  PINE_UNREACHABLE;
  return vec3{0.0f};
}
float SpecularReflectionBSDF::pdf(vec3, vec3, const NodeEvalCtx&) const {
  PINE_UNREACHABLE;
  return 0.0f;
}

psl::optional<BSDFSample> SpecularRefrectionBSDF::sample(vec3 wi, float u1, vec2,
                                                         const NodeEvalCtx& nc) const {
  float etap = eta(nc);
  if (CosTheta(wi) < 0)
    etap = 1.0f / etap;
  float fr = FrDielectric(AbsCosTheta(wi), etap);

  auto bs = BSDFSample{};
  bs.f = albedo(nc);
  if (u1 < fr)
    bs.wo = Reflect(wi, vec3{0, 0, 1});
  else if (!Refract(wi, vec3{0, 0, 1}, etap, bs.wo))
    return psl::nullopt;
  bs.pdf = psl::max(AbsCosTheta(bs.wo), Epsilon);
  return bs;
}

vec3 SpecularRefrectionBSDF::f(vec3, vec3, const NodeEvalCtx&) const {
  PINE_UNREACHABLE;
  return vec3{0.0f};
}
float SpecularRefrectionBSDF::pdf(vec3, vec3, const NodeEvalCtx&) const {
  PINE_UNREACHABLE;
  return 0.0f;
}

}  // namespace pine