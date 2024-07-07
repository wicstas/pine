#pragma once

#include <pine/core/sampling.h>

namespace pine {

inline float CosTheta(vec3 w) {
  return w.z;
}
inline float Cos2Theta(vec3 w) {
  return sqr(w.z);
}
inline float AbsCosTheta(vec3 w) {
  return psl::abs(w.z);
}
inline float Sin2Theta(vec3 w) {
  return 1.0f - Cos2Theta(w);
}
inline float SinTheta(vec3 w) {
  return psl::sqrt(Sin2Theta(w));
}
inline float TanTheta(vec3 w) {
  return CosTheta(w) == 0.0f ? 0.0f : SinTheta(w) / CosTheta(w);
}
inline float Tan2Theta(vec3 w) {
  return Sin2Theta(w) / psl::max(Cos2Theta(w), epsilon);
}
inline float CosPhi(vec3 w) {
  float sinTheta = SinTheta(w);
  return (sinTheta == 0) ? 1 : psl::clamp(w.x / sinTheta, -1.0f, 1.0f);
}
inline float SinPhi(vec3 w) {
  float sinTheta = SinTheta(w);
  return (sinTheta == 0) ? 1 : psl::clamp(w.y / sinTheta, -1.0f, 1.0f);
}

inline bool SameHemisphere(vec3 w0, vec3 w1) {
  return w0.z * w1.z > 0.0f;
}
inline vec3 FaceForward(vec3 v, vec3 n) {
  return (dot(v, n) < 0.0f) ? -v : v;
}
inline vec3 FaceNormal(vec3 v) {
  return v.z < 0.0f ? -v : v;
}

inline vec3 Reflect(vec3 w) {
  return vec3(-w.x, -w.y, w.z);
}
inline vec3 Reflect(vec3 wi, vec3 n) {
  return 2.0f * dot(wi, n) * n - wi;
}
inline bool Refract(vec3 wi, vec3 n, float eta, vec3& wt, float* etap = nullptr) {
  float cosThetaI = dot(n, wi);
  if (cosThetaI < 0) {
    eta = 1.0f / eta;
    cosThetaI = -cosThetaI;
    n = -n;
  }

  float sin2ThetaI = psl::max(0.0f, 1.0f - sqr(cosThetaI));
  float sin2ThetaT = sin2ThetaI / sqr(eta);
  if (sin2ThetaT >= 1)
    return false;

  float cosThetaT = psl::sqrt(1.0f - sin2ThetaT);

  wt = -wi / eta + (cosThetaI / eta - cosThetaT) * n;
  if (etap)
    *etap = eta;
  return true;
}

inline float FrDielectric(float cosThetaI, float eta) {
  if (cosThetaI < 0) {
    eta = 1 / eta;
    cosThetaI = -cosThetaI;
  }

  float sin2ThetaI = 1.0f - sqr(cosThetaI);
  float sin2ThetaT = sin2ThetaI / sqr(eta);
  if (sin2ThetaT >= 1.0f)
    return 1.0f;
  float cosThetaT = psl::sqrt(1.0f - sin2ThetaT);

  float rParl = (eta * cosThetaI - cosThetaT) / (eta * cosThetaI + cosThetaT);
  float rPerp = (cosThetaI - eta * cosThetaT) / (cosThetaI + eta * cosThetaT);
  return (sqr(rParl) + sqr(rPerp)) / 2.0f;
}

inline vec3 FrSchlick(vec3 F0, float cosTheta) {
  return F0 + (vec3(1.0f) - F0) * psl::pow(1.0f - cosTheta, 5.0f);
}

struct TrowbridgeReitzDistribution {
public:
  TrowbridgeReitzDistribution() = default;

  TrowbridgeReitzDistribution(float alphaX, float alphaY) : alphaX(alphaX), alphaY(alphaY){};
  float D(vec3 wm) const {
    float tan2Theta = Tan2Theta(wm);
    float cos4Theta = sqr(Cos2Theta(wm));
    if (cos4Theta < 1e-6f)
      return 0.0f;
    float e = tan2Theta * (sqr(CosPhi(wm) / alphaX) + sqr(SinPhi(wm) / alphaY));
    return 1.0f / (Pi * alphaX * alphaY * cos4Theta * sqr(1 + e));
  }
  float Lambda(vec3 w) const {
    float tan2Theta = Tan2Theta(w);
    float alpha2 = sqr(CosPhi(w) * alphaX) + sqr(SinPhi(w) * alphaY);
    return (psl::sqrt(1.0f + alpha2 * tan2Theta) - 1.0f) / 2.0f;
  }

  float G1(vec3 w) const {
    return 1.0f / (1.0f + Lambda(w));
  }
  float G(vec3 wi, vec3 wo) const {
    return 1.0f / (1.0f + Lambda(wi) + Lambda(wo));
  }
  float D_G(vec3 wi, vec3 wm, vec3 wo) const {
    return D(wm) * G(wi, wo);
  }

  float D(vec3 w, vec3 wm) const {
    return G1(w) / AbsCosTheta(w) * D(wm) * absdot(w, wm);
  }

  float pdf(vec3 w, vec3 wm) const {
    return psl::max(D(w, wm), epsilon);
  }

  vec3 SampleWm(vec3 w, vec2 u) const {
    vec3 wh = normalize(vec3(alphaX * w.x, alphaY * w.y, w.z));
    if (wh.z < 0.0f)
      wh = -wh;

    vec3 T1 = (wh.z < 0.99999f) ? normalize(cross(vec3(0, 0, 1), wh)) : vec3(1, 0, 0);
    vec3 T2 = cross(wh, T1);

    vec2 p = sample_disk_polar(u);

    float h = psl::sqrt(1.0f - sqr(p.x));
    p.y = psl::lerp((1.0f + wh.z) / 2, h, p.y);

    float pz = psl::sqrt(psl::max(0.0f, 1.0f - length_squared(p)));
    vec3 nh = p.x * T1 + p.y * T2 + pz * wh;

    return normalize(vec3(alphaX * nh.x, alphaY * nh.y, psl::max(1e-6f, nh.z)));
  }

  float alphaX, alphaY;
};

}  // namespace pine
