#include <pine/core/scattering.h>
#include <pine/core/geometry.h>
#include <pine/core/sampler.h>
#include <pine/core/bxdf.h>

namespace pine {

// ================================================
// DiffuseBSDF
// ================================================
psl::optional<BSDFSample> DiffuseBSDF::sample(vec3 wi, Sampler& sampler) const {
  BSDFSample bs;

  vec3 wo = cosine_weighted_hemisphere(sampler.get2d());
  if (CosTheta(wi) < 0)
    wo = -wo;

  bs.wo = wo;
  bs.pdf = AbsCosTheta(bs.wo) / Pi;
  bs.f = albedo / Pi;
  return bs;
}
vec3 DiffuseBSDF::f(vec3 wi, vec3 wo) const {
  if (!SameHemisphere(wi, wo))
    return vec3(0.0f);
  return albedo / Pi;
}
float DiffuseBSDF::pdf(vec3 wi, vec3 wo) const {
  if (!SameHemisphere(wi, wo))
    return 0.0f;
  return AbsCosTheta(wo) / Pi;
}

// ================================================
// ConductorBSDF
// ================================================
psl::optional<BSDFSample> ConductorBSDF::sample(vec3 wi, Sampler& sampler) const {
  BSDFSample bs;

  auto alpha = sqr(roughness);
  if (alpha < 1e-4f) {
    bs.wo = Reflect(wi);
    bs.f = FrSchlick(albedo, AbsCosTheta(bs.wo)) / AbsCosTheta(bs.wo);
    bs.pdf = 1.0f;
    bs.is_delta = true;
    return bs;
  }

  auto distrib = TrowbridgeReitzDistribution(alpha, alpha);
  auto wm = distrib.SampleWm(wi, sampler.get2d());
  auto wo = Reflect(wi, wm);
  if (!SameHemisphere(wi, wo))
    return psl::nullopt;

  auto fr = FrSchlick(albedo, absdot(wi, wm));
  bs.wo = wo;
  bs.pdf = distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
  bs.f = fr * (distrib.D_G(wo, wm, wi) / (4 * CosTheta(wi) * CosTheta(wo)));

  return bs;
}
vec3 ConductorBSDF::f(vec3 wi, vec3 wo) const {
  if (!SameHemisphere(wi, wo))
    return {};

  auto alpha = sqr(roughness);
  CHECK(alpha >= 1e-4f);
  auto distrib = TrowbridgeReitzDistribution(alpha, alpha);

  auto wm = normalize(wi + wo);
  if (wm.is_zero())
    return {};

  auto fr = FrSchlick(albedo, absdot(wi, wm));
  return fr * (distrib.D_G(wo, wm, wi) / (4 * AbsCosTheta(wo) * AbsCosTheta(wi)));
}
float ConductorBSDF::pdf(vec3 wi, vec3 wo) const {
  if (!SameHemisphere(wi, wo))
    return {};

  auto alpha = sqr(roughness);
  CHECK(alpha >= 1e-4f);
  auto distrib = TrowbridgeReitzDistribution(alpha, alpha);

  auto wm = normalize(wi + wo);
  if (wm.is_zero())
    return {};

  wm = FaceNormal(wm);
  return distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
}

// ================================================
// RefractiveBSDF
// ================================================
psl::optional<BSDFSample> RefractiveBSDF::sample(vec3 wi, Sampler& sampler) const {
  BSDFSample bs;
  auto alpha = sqr(roughness);
  if (alpha < 1e-4f) {
    bs.wo = Reflect(wi);
    bs.f = albedo;
    bs.pdf = AbsCosTheta(bs.wo);
    bs.is_delta = true;
    return bs;
  }
  auto distrib = TrowbridgeReitzDistribution(alpha, alpha);
  auto wm = distrib.SampleWm(wi, sampler.get2d());

  vec3 wo = Reflect(wi, wm);
  if (!SameHemisphere(wi, wo))
    return psl::nullopt;

  bs.wo = wo;
  bs.pdf = distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
  bs.f = albedo * (distrib.D_G(wo, wm, wi) / (4 * CosTheta(wi) * CosTheta(wo)));
  return bs;
}
vec3 RefractiveBSDF::f(vec3 wi, vec3 wo) const {
  auto alpha = sqr(roughness);
  CHECK(alpha >= 1e-4f);
  auto distrib = TrowbridgeReitzDistribution(alpha, alpha);

  auto cosThetaO = CosTheta(wo), cosThetaI = CosTheta(wi);
  auto reflect = cosThetaI * cosThetaO > 0;
  if (!reflect)
    return {};

  auto wm = FaceNormal(normalize(wo + wi));
  if (dot(wm, wo) * cosThetaO <= 0 || dot(wm, wi) * cosThetaI <= 0)
    return {};

  return albedo * (distrib.D_G(wi, wm, wo) / psl::abs(4 * cosThetaI * cosThetaO));
}
float RefractiveBSDF::pdf(vec3 wi, vec3 wo) const {
  auto alpha = sqr(roughness);
  CHECK(alpha >= 1e-4f);
  auto distrib = TrowbridgeReitzDistribution(alpha, alpha);

  auto cosThetaO = CosTheta(wo), cosThetaI = CosTheta(wi);
  auto reflect = cosThetaI * cosThetaO > 0;
  if (!reflect)
    return 0.0f;

  auto wm = FaceNormal(normalize(wo + wi));
  if (dot(wm, wo) * cosThetaO <= 0 || dot(wm, wi) * cosThetaI <= 0)
    return {};

  return distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
}

// ================================================
// RefractiveDielectricBSDF
// ================================================
psl::optional<BSDFSample> RefractiveDielectricBSDF::sample(vec3 wi, Sampler& sampler) const {
  BSDFSample bs;
  auto fr = FrDielectric(CosTheta(wi), ior);

  auto alpha = sqr(roughness);
  if (alpha < 1e-4f) {
    if (sampler.get1d() < fr) {
      bs.wo = Reflect(wi);
      bs.f = albedo * (fr / AbsCosTheta(bs.wo));
      bs.pdf = fr;
      bs.is_delta = true;
    } else {
      auto& wo = bs.wo;
      if (!Refract(wi, vec3(0, 0, 1), ior, wo))
        return psl::nullopt;
      bs.f = albedo * ((1 - fr) / AbsCosTheta(wo));
      bs.pdf = 1 - fr;
      bs.is_delta = true;
    }
    return bs;
  }
  auto distrib = TrowbridgeReitzDistribution(alpha, alpha);
  auto wm = distrib.SampleWm(wi, sampler.get2d());

  if (sampler.get1d() < fr) {
    vec3 wo = Reflect(wi, wm);
    if (!SameHemisphere(wi, wo))
      return psl::nullopt;

    bs.wo = wo;
    bs.pdf = fr * distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
    bs.f = albedo * (fr * distrib.D_G(wo, wm, wi) / (4 * CosTheta(wi) * CosTheta(wo)));
  } else {
    auto& wo = bs.wo;
    auto eta = 1.0f;
    if (!Refract(wi, wm, ior, wo, &eta))
      return psl::nullopt;
    auto denom = sqr(dot(wo, wm) + dot(wi, wm) / eta);
    bs.pdf = (1 - fr) * distrib.pdf(wi, wm) * absdot(wo, wm) / denom;
    bs.f = albedo * ((1 - fr) * distrib.D(wm) * distrib.G(wi, wo) *
                     psl::abs(dot(wo, wm) * dot(wi, wm) / (denom * CosTheta(wi) * CosTheta(wo))));
  }
  return bs;
}
vec3 RefractiveDielectricBSDF::f(vec3 wi, vec3 wo) const {
  auto alpha = sqr(roughness);
  CHECK(alpha >= 1e-4f);
  auto distrib = TrowbridgeReitzDistribution(alpha, alpha);

  auto cosThetaO = CosTheta(wo), cosThetaI = CosTheta(wi);
  auto reflect = cosThetaI * cosThetaO > 0;
  auto eta = 1;
  if (!reflect)
    eta = cosThetaI > 0 ? ior : 1 / ior;

  auto wm = FaceNormal(normalize(wo * eta + wi));
  if (dot(wm, wo) * cosThetaO <= 0 || dot(wm, wi) * cosThetaI <= 0)
    return {};

  auto fr = FrDielectric(dot(wi, wm), ior);
  if (reflect) {
    return albedo * (fr * distrib.D_G(wi, wm, wo) / psl::abs(4 * cosThetaI * cosThetaO));
  } else {
    auto denom = sqr(dot(wo, wm) + dot(wi, wm) / eta) * cosThetaI * cosThetaO;
    return albedo * ((1 - fr) * distrib.D(wm) * distrib.G(wi, wo) *
                     psl::abs(dot(wo, wm) * dot(wi, wm) / denom));
  }
}
float RefractiveDielectricBSDF::pdf(vec3 wi, vec3 wo) const {
  auto alpha = sqr(roughness);
  CHECK(alpha >= 1e-4f);
  auto distrib = TrowbridgeReitzDistribution(alpha, alpha);

  auto cosThetaO = CosTheta(wo), cosThetaI = CosTheta(wi);
  auto reflect = cosThetaI * cosThetaO > 0;
  auto eta = 1;
  if (!reflect)
    eta = cosThetaI > 0 ? ior : 1 / ior;

  auto wm = FaceNormal(normalize(wo * eta + wi));
  if (dot(wm, wo) * cosThetaO <= 0 || dot(wm, wi) * cosThetaI <= 0)
    return {};

  auto fr = FrDielectric(dot(wi, wm), ior);
  if (reflect) {
    return fr * distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
  } else {
    auto denom = sqr(dot(wo, wm) + dot(wi, wm) / eta);
    auto dwm_dwo = absdot(wo, wm) / denom;
    return (1 - fr) * distrib.pdf(wi, wm) * dwm_dwo;
  }
}

// ================================================
// DiffusiveDielectricBSDF
// ================================================
psl::optional<BSDFSample> DiffusiveDielectricBSDF::sample(vec3 wi, Sampler& sampler) const {
  BSDFSample bs;

  auto fr = FrDielectric(CosTheta(wi), ior);

  auto alpha = sqr(roughness);
  if (alpha < 1e-4f) {
    if (sampler.get1d() < fr) {
      bs.wo = Reflect(wi);
      bs.f = vec3(fr);
      bs.pdf = fr * AbsCosTheta(bs.wo);
      bs.is_delta = true;
    } else {
      bs.wo = cosine_weighted_hemisphere(sampler.get2d());
      bs.f = albedo * ((1 - fr) / Pi);
      bs.pdf = (1 - fr) * AbsCosTheta(bs.wo) / Pi;
    }
    return bs;
  }
  auto distrib = TrowbridgeReitzDistribution(alpha, alpha);
  auto wm = distrib.SampleWm(wi, sampler.get2d());

  if (sampler.get1d() < fr) {
    auto wo = Reflect(wi, wm);
    if (!SameHemisphere(wi, wo))
      return psl::nullopt;

    bs.wo = wo;
    bs.f = vec3(fr * distrib.D_G(wi, wm, wo) / (4 * CosTheta(wi) * CosTheta(wo)));
    bs.pdf = fr * distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
  } else {
    bs.wo = cosine_weighted_hemisphere(sampler.get2d());
    bs.f = albedo * ((1 - fr) / Pi);
    bs.pdf = AbsCosTheta(bs.wo) * (1 - fr) / Pi;
  }
  return bs;
}
vec3 DiffusiveDielectricBSDF::f(vec3 wi, vec3 wo) const {
  if (!SameHemisphere(wi, wo))
    return {};
  auto alpha = sqr(roughness);
  auto distrib = TrowbridgeReitzDistribution(alpha, alpha);

  auto cosThetaO = CosTheta(wo), cosThetaI = CosTheta(wi);

  auto wm = FaceNormal(normalize(wo + wi));
  if (dot(wm, wo) * cosThetaO <= 0 || dot(wm, wi) * cosThetaI <= 0)
    return {};

  auto fr = FrDielectric(dot(wi, wm), ior);
  auto diffused = albedo * (1 - fr) / Pi;
  if (alpha < 1e-4f)
    return diffused;
  auto reflected = fr * distrib.D_G(wo, wm, wi) / psl::abs(4 * cosThetaI * cosThetaO);
  return vec3(reflected) + diffused;
}
float DiffusiveDielectricBSDF::pdf(vec3 wi, vec3 wo) const {
  if (!SameHemisphere(wi, wo))
    return {};
  auto alpha = sqr(roughness);
  auto distrib = TrowbridgeReitzDistribution(alpha, alpha);

  auto cosThetaO = CosTheta(wo), cosThetaI = CosTheta(wi);

  auto wm = FaceNormal(normalize(wo + wi));
  if (dot(wm, wo) * cosThetaO <= 0 || dot(wm, wi) * cosThetaI <= 0)
    return {};

  auto fr = FrDielectric(dot(wi, wm), ior);
  auto pt = (1 - fr) * AbsCosTheta(wo) / Pi;
  if (alpha < 1e-4f)
    return pt;
  auto pr = fr * distrib.pdf(wi, wm) / (4 * absdot(wi, wm));
  return pr + pt;
}

// ================================================
// BSSRDF
// ================================================
psl::optional<SubsurfaceSample> BSSRDF::sample_p(const BxdfSampleCtx& bc, Sampler& sampler) const {
  auto p = bc.p;
  auto w = -bc.wi;
  if (!Refract(bc.wi, bc.n, ior, w))
    return psl::nullopt;

  auto channel = int(sampler.randf() * 3);
  auto beta = vec3(0);
  beta[channel] = 3;
  auto sigma_t_inv = 1 / sigma_s[channel];

  for (int i = 0;; i++) {
    auto ray = i == 0 ? spawn_ray(p, bc.n, w) : Ray(p, w);
    auto it = SurfaceInteraction();
    if (!bc.it.shape->intersect(ray, it))
      return psl::nullopt;
    auto t = -psl::log(1 - sampler.get1d()) * sigma_t_inv;
    if (ray.tmax < t) {
      return SubsurfaceSample{-w, it.p, it.n, beta};
    } else {
      p = ray(t);
      w = uniform_sphere(sampler.get2d());
    }
  }
  return psl::nullopt;
}
psl::optional<BSDFSample> BSSRDF::sample(vec3 wi, Sampler& sampler) const {
  BSDFSample bs;

  auto wo = cosine_weighted_hemisphere(sampler.get2d());
  if (CosTheta(wi) > 0)
    wo = -wo;

  bs.wo = wo;
  bs.pdf = AbsCosTheta(bs.wo) / Pi;
  bs.f = albedo / Pi;
  return bs;
}
vec3 BSSRDF::f(vec3, vec3) const {
  return albedo / Pi;
}
float BSSRDF::pdf(vec3, vec3 wo) const {
  return AbsCosTheta(wo) / Pi;
}

void BXDF::sample_p(vec3& beta, const BxdfSampleCtx& bc, Sampler& sampler) {
  if (auto ss = dispatch([&](auto&& x) { return x.sample_p(bc, sampler); })) {
    beta = ss->beta;
    bc.it.p = ss->p;
    bc.it.n = ss->n;
    bc.it.compute_transformation();
    wi = bc.it.to_local(ss->wi);
  }
}

}  // namespace pine