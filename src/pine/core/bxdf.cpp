#include <pine/core/scattering.h>
#include <pine/core/bxdf.h>

namespace pine {

psl::optional<BSDFSample> DiffuseBSDF::sample(vec3 wi, float, vec2 u, const BxdfEvalCtx& bc) const {
  BSDFSample bs;

  vec3 wo = cosine_weighted_hemisphere(u);
  if (CosTheta(wi) < 0)
    wo = -wo;

  bs.wo = wo;
  bs.pdf = AbsCosTheta(bs.wo) / Pi;
  bs.f = albedo(bc) / Pi;
  return bs;
}
vec3 DiffuseBSDF::f(vec3 wi, vec3 wo, const BxdfEvalCtx& bc) const {
  if (!SameHemisphere(wi, wo))
    return vec3(0.0f);
  return albedo(bc) / Pi;
}
float DiffuseBSDF::pdf(vec3 wi, vec3 wo, const BxdfEvalCtx&) const {
  if (!SameHemisphere(wi, wo))
    return 0.0f;
  return AbsCosTheta(wo) / Pi;
}

psl::optional<BSDFSample> ConductorBSDF::sample(vec3 wi, float, vec2 u2,
                                                const BxdfEvalCtx& bc) const {
  BSDFSample bs;

  float alpha = psl::clamp(psl::sqr(roughness(bc)), bc.bounded_roughness(0.001f), 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);
  vec3 wm = distrib.SampleWm(wi, u2);

  vec3 wo = Reflect(wi, wm);
  if (!SameHemisphere(wi, wo))
    return psl::nullopt;

  vec3 fr = FrSchlick(albedo(bc), AbsCosTheta(wm));

  bs.wo = wo;
  bs.pdf = distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
  bs.f = fr * distrib.D(wm) * distrib.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo));

  return bs;
}
vec3 ConductorBSDF::f(vec3 wi, vec3 wo, const BxdfEvalCtx& bc) const {
  if (!SameHemisphere(wi, wo))
    return {};

  float alpha = psl::clamp(psl::sqr(roughness(bc)), bc.bounded_roughness(0.001f), 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);

  vec3 wh = normalize(wi + wo);

  vec3 fr = FrSchlick(albedo(bc), AbsCosTheta(wh));
  return fr * distrib.D(wh) * distrib.G(wo, wi) / (4 * AbsCosTheta(wo) * AbsCosTheta(wi));
}
float ConductorBSDF::pdf(vec3 wi, vec3 wo, const BxdfEvalCtx& bc) const {
  if (!SameHemisphere(wi, wo))
    return {};

  float alpha = psl::clamp(psl::sqr(roughness(bc)), bc.bounded_roughness(0.001f), 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);

  vec3 wh = normalize(wi + wo);

  return distrib.pdf(wi, wh) / (4 * absdot(wi, wh));
}

psl::optional<BSDFSample> RefractiveDielectricBSDF::sample(vec3 wi, float u1, vec2 u2,
                                                           const BxdfEvalCtx& bc) const {
  BSDFSample bs;
  float etap = eta(bc);
  if (CosTheta(wi) < 0)
    etap = 1.0f / etap;
  float fr = FrDielectric(AbsCosTheta(wi), etap);

  float alpha = psl::clamp(psl::sqr(roughness(bc)), bc.bounded_roughness(0.001f), 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);
  vec3 wm = distrib.SampleWm(wi, u2);

  if (u1 < fr) {
    vec3 wo = Reflect(wi, wm);
    if (!SameHemisphere(wi, wo))
      return psl::nullopt;

    bs.wo = wo;
    bs.pdf = fr * distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
    bs.f = albedo(bc) * fr * distrib.D(wm) * distrib.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo));
  } else {
    vec3 wo;
    if (!Refract(wi, wm, etap, wo))
      return psl::nullopt;
    float cosThetaO = CosTheta(wo), cosThetaI = CosTheta(wi);
    bs.wo = wo;
    float denom = psl::sqr(dot(wo, wm) + dot(wi, wm) / etap);
    float dwm_dwo = absdot(wo, wm) / denom;
    bs.pdf = distrib.pdf(wi, wm) * dwm_dwo;
    bs.f = albedo(bc) * distrib.D(wm) * distrib.G(wi, wo) *
           psl::abs(dot(wo, wm) * dot(wi, wm) / denom / cosThetaI / cosThetaO);
  }
  return bs;
}
vec3 RefractiveDielectricBSDF::f(vec3 wi, vec3 wo, const BxdfEvalCtx& bc) const {
  float alpha = psl::clamp(psl::sqr(roughness(bc)), bc.bounded_roughness(0.001f), 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);

  float cosThetaO = CosTheta(wo), cosThetaI = CosTheta(wi);
  bool reflect = cosThetaI * cosThetaO > 0;
  float etap = eta(bc);
  if (!reflect)
    etap = 1.0f / etap;

  vec3 wm = FaceForward(normalize(wi * etap + wo), vec3(0, 0, 1));
  if (dot(wm, wo) * cosThetaI < 0.0f || dot(wm, wi) * cosThetaO < 0.0f)
    return {};

  if (reflect) {
    return albedo(bc) * distrib.D(wm) * distrib.G(wo, wi) / (4 * cosThetaI * cosThetaO);
  } else {
    float denom = psl::sqr(dot(wo, wm) + dot(wi, wm) / etap) * cosThetaI * cosThetaO;
    return albedo(bc) * distrib.D(wm) * distrib.G(wi, wo) *
           psl::abs(dot(wo, wm) * dot(wi, wm) / psl::max(denom, epsilon));
  }
}
float RefractiveDielectricBSDF::pdf(vec3 wi, vec3 wo, const BxdfEvalCtx& bc) const {
  float alpha = psl::clamp(psl::sqr(roughness(bc)), bc.bounded_roughness(0.001f), 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);

  float cosThetaO = CosTheta(wo), cosThetaI = CosTheta(wi);
  bool reflect = cosThetaI * cosThetaO > 0;
  float etap = eta(bc);
  if (!reflect)
    etap = 1.0f / etap;

  vec3 wm = FaceForward(normalize(wi * etap + wo), vec3(0, 0, 1));
  if (dot(wm, wo) * cosThetaI < 0.0f || dot(wm, wi) * cosThetaO < 0.0f)
    return 0.0f;

  if (reflect) {
    return distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
  } else {
    float denom = psl::sqr(dot(wo, wm) + dot(wi, wm) / etap);
    float dwm_dwo = absdot(wo, wm) / denom;
    return distrib.pdf(wi, wm) * dwm_dwo;
  }
}

psl::optional<BSDFSample> DiffusiveDielectricBSDF::sample(vec3 wi, float u1, vec2 u2,
                                                          const BxdfEvalCtx& bc) const {
  BSDFSample bs;
  float etap = eta(bc);
  float fr = FrDielectric(AbsCosTheta(wi), etap);

  float alpha = psl::clamp(psl::sqr(roughness(bc)), bc.bounded_roughness(0.001f), 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);
  vec3 wm = distrib.SampleWm(wi, u2);

  if (u1 < fr) {
    vec3 wo = Reflect(wi, wm);
    if (!SameHemisphere(wi, wo))
      return psl::nullopt;

    bs.wo = wo;
    bs.pdf = distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
    bs.f = vec3(distrib.D(wm) * distrib.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo)));
  } else {
    vec3 wo = cosine_weighted_hemisphere(u2);
    bs.wo = wo;
    bs.pdf = AbsCosTheta(bs.wo) / Pi;
    bs.f = albedo(bc) / Pi;
  }
  if (CosTheta(wi) < 0)
    bs.wo = -bs.wo;
  return bs;
}
vec3 DiffusiveDielectricBSDF::f(vec3 wi, vec3 wo, const BxdfEvalCtx& bc) const {
  float alpha = psl::clamp(psl::sqr(roughness(bc)), bc.bounded_roughness(0.001f), 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);

  float cosThetaO = CosTheta(wo), cosThetaI = CosTheta(wi);
  float etap = eta(bc);

  vec3 wm = FaceForward(normalize(wi * etap + wo), vec3(0, 0, 1));
  float fr = FrDielectric(AbsCosTheta(wi), etap);

  auto f0 = vec3(fr * distrib.D(wm) * distrib.G(wo, wi) / (4 * cosThetaI * cosThetaO));
  auto f1 = (1 - fr) * albedo(bc) / Pi;
  return f0 + f1;
}
float DiffusiveDielectricBSDF::pdf(vec3 wi, vec3 wo, const BxdfEvalCtx& bc) const {
  float alpha = psl::clamp(psl::sqr(roughness(bc)), bc.bounded_roughness(0.001f), 1.0f);
  TrowbridgeReitzDistribution distrib(alpha, alpha);

  float cosThetaO = CosTheta(wo), cosThetaI = CosTheta(wi);
  float etap = eta(bc);

  vec3 wm = FaceForward(normalize(wi * etap + wo), vec3(0, 0, 1));
  if (dot(wm, wo) * cosThetaI < 0.0f || dot(wm, wi) * cosThetaO < 0.0f)
    return 0.0f;

  float fr = FrDielectric(AbsCosTheta(wi), eta(bc));

  auto pdf0 = fr * distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
  auto pdf1 = (1 - fr) * AbsCosTheta(wo);
  return pdf0 + pdf1;
}

psl::optional<BSDFSample> SpecularReflectionBSDF::sample(vec3 wi, float, vec2,
                                                         const BxdfEvalCtx& bc) const {
  auto bs = BSDFSample();
  bs.wo = Reflect(wi, vec3(0, 0, 1));
  bs.f = albedo(bc) / psl::max(AbsCosTheta(bs.wo), epsilon);
  bs.pdf = 1.0f;
  return bs;
}

psl::optional<BSDFSample> SpecularRefrectionBSDF::sample(vec3 wi, float, vec2,
                                                         const BxdfEvalCtx& bc) const {
  // float etap = eta(bc);
  // if (CosTheta(wi) < 0)
  //   etap = 1.0f / etap;
  // float fr = FrDielectric(AbsCosTheta(wi), etap);

  auto bs = BSDFSample();
  // if (u1 < fr)
  //   bs.wo = Reflect(wi, vec3(0, 0, 1));
  // else if (!Refract(wi, vec3(0, 0, 1), etap, bs.wo))
  //   return psl::nullopt;
  // bs.f = albedo(bc) / psl::max(AbsCosTheta(bs.wo), epsilon);
  // bs.pdf = 1.0f;
  bs.wo = -wi;
  bs.f = albedo(bc) / psl::max(AbsCosTheta(bs.wo), epsilon);
  bs.pdf = 1.0f;
  return bs;
}

vec3 wavelength_to_xyz(float wavelength) {
  auto g = [x = wavelength](float u, float t1, float t2) {
    return x < u ? psl::exp(-t1 * t1 * sqr(x - u) / 2) : psl::exp(-t2 * t2 * sqr(x - u) / 2);
  };
  auto x = 1.056f * g(599.8f, 0.0264f, 0.0323f) + 0.362f * g(442.0f, 0.0624f, 0.0374f) -
           0.065f * g(501.1f, 0.049f, 0.0382f);
  auto y = 0.821f * g(568.8f, 0.0213f, 0.0247f) + 0.286f * g(530.9f, 0.0613f, 0.0322f);
  auto z = 1.217f * g(437.0f, 0.0845f, 0.0278f) + 0.681f * g(459.0f, 0.0385f, 0.0725f);
  return {x, y, z};
}
vec3 xyz_to_rgb(vec3 x) {
  // clang-format off
  return mat3(3.2406f, -1.5372f, -0.4986f, 
             -0.9689f,  1.8758f,  0.0415f, 
              0.0557f, -0.204f ,  1.057f) *
         x;
  // clang-format on
}
vec3 wavelength_to_rgb(float x) {
  return xyz_to_rgb(wavelength_to_xyz(x));
}
float wavelength_to_ior_glass(float x) {
  x = sqr(x / 1000.0f);
  return psl::sqrt(1 + 1.03961212f / (1 - 0.00600069867f / x) +
                   0.231792344f / (1 - 0.0200179144f / x) + 1.01046945f / (1 - 103.560653f / x));
}
psl::optional<BSDFSample> DispersionGlassBSDF::sample(vec3 wi, float u1, vec2 u2,
                                                      const BxdfEvalCtx& bc) const {
  auto sampled_wavelength = lerp(u1, 420.0f, 680.0f);
  //   auto etap = wavelength_to_ior_glass(sampled_wavelength);
  auto etap = lerp(u1, 1.35f, 1.4f);
  if (CosTheta(wi) < 0)
    etap = 1.0f / etap;
  auto fr = FrDielectric(AbsCosTheta(wi), etap);

  auto bs = BSDFSample();
  if (u2[0] < fr)
    bs.wo = Reflect(wi, vec3(0, 0, 1));
  else if (!Refract(wi, vec3(0, 0, 1), etap, bs.wo))
    return psl::nullopt;
  bs.f = wavelength_to_rgb(sampled_wavelength) * albedo(bc) / psl::max(AbsCosTheta(bs.wo), epsilon);
  bs.pdf = 1.0f;
  return bs;
}

}  // namespace pine