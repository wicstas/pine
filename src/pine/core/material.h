#pragma once
#include <pine/core/interaction.h>
#include <pine/core/scattering.h>
#include <pine/core/sampler.h>
#include <pine/core/bxdf.h>

#include <psl/variant.h>

namespace pine {

struct LeEvalCtx : NodeEvalCtx {
  LeEvalCtx(vec3 p, vec3 n, vec2 uv, vec3 wo) : NodeEvalCtx(p, n, uv), wo(wo) {}
  LeEvalCtx(const SurfaceInteraction& it, vec3 wo) : NodeEvalCtx(it.p, it.n, it.uv), wo(wo) {}

  vec3 wo;
};

struct EmissiveMaterial {
  EmissiveMaterial(Node3f color) : color{MOVE(color)} {}

  BXDF sample_bxdf(const BxdfSampleCtx&, Sampler&) const { PINE_UNREACHABLE; }
  vec3 le(const LeEvalCtx& ec) const {
    if (dot(ec.wo, ec.n) < 0.0f) return vec3(0.0f);
    return color.eval(ec);
  }

  Node3f color;
};

struct DiffuseMaterial {
  DiffuseMaterial(Node3f albedo) : albedo(MOVE(albedo)) {}

  BXDF sample_bxdf(const BxdfSampleCtx& bc, Sampler&) const { return DiffuseBSDF(albedo(bc)); }
  vec3 le(const LeEvalCtx&) const { return {}; }

  Node3f albedo;
};

struct MetalMaterial {
  MetalMaterial(Node3f albedo, Nodef roughness)
      : albedo(MOVE(albedo)), roughness(MOVE(roughness)) {}

  BXDF sample_bxdf(const BxdfSampleCtx& bc, Sampler&) const {
    return ConductorBSDF(albedo(bc), psl::max(roughness(bc), bc.min_roughness));
  }
  vec3 le(const LeEvalCtx&) const { return {}; }

  Node3f albedo;
  Nodef roughness;
};

struct GlossyMaterial {
  GlossyMaterial(Node3f albedo, Nodef roughness, Nodef ior = 1.4f)
      : albedo(MOVE(albedo)), roughness(MOVE(roughness)), ior(MOVE(ior)) {}

  BXDF sample_bxdf(const BxdfSampleCtx& bc, Sampler&) const {
    return DiffusiveDielectricBSDF(albedo(bc), psl::max(roughness(bc), bc.min_roughness), ior(bc));
  }
  vec3 le(const LeEvalCtx&) const { return {}; }

  Node3f albedo;
  Nodef roughness;
  Nodef ior;
};

struct GlassMaterial {
  GlassMaterial(Node3f albedo, Nodef roughness, Nodef ior = 1.4f)
      : albedo(MOVE(albedo)), roughness(MOVE(roughness)), ior(MOVE(ior)) {}

  BXDF sample_bxdf(const BxdfSampleCtx& bc, Sampler&) const {
    return RefractiveDielectricBSDF(albedo(bc), psl::max(roughness(bc), bc.min_roughness), ior(bc));
  }
  vec3 le(const LeEvalCtx&) const { return {}; }

  Node3f albedo;
  Nodef roughness;
  Nodef ior;
};

struct UberMaterial {
  UberMaterial(Node3f albedo, Nodef roughness, Nodef metallic = 0.0f, Nodef transmission = 0.0f,
               float ior = 1.45f)
      : albedo(MOVE(albedo)),
        roughness(MOVE(roughness)),
        metallic(MOVE(metallic)),
        transmission(MOVE(transmission)),
        ior(ior) {}

  BXDF sample_bxdf(const BxdfSampleCtx& bc, Sampler&) const;
  vec3 le(const LeEvalCtx&) const { return {}; }

  Node3f albedo;
  Nodef roughness;
  Nodef metallic;
  Nodef transmission;
  float ior;
};

struct SubsurfaceMaterial {
  SubsurfaceMaterial(Node3f albedo, Nodef roughness, vec3 sigma_s)
      : albedo(MOVE(albedo)), roughness(MOVE(roughness)), sigma_s(sigma_s) {}

  BXDF sample_bxdf(const BxdfSampleCtx& bc, Sampler&) const;
  vec3 le(const LeEvalCtx&) const { return {}; }

  Node3f albedo;
  Nodef roughness;
  vec3 sigma_s;
  float ior = 1.4f;
};

struct Material : psl::variant<EmissiveMaterial, DiffuseMaterial, MetalMaterial, GlossyMaterial,
                               GlassMaterial, UberMaterial, SubsurfaceMaterial> {
 public:
  using variant::variant;

  BXDF sample_bxdf(const BxdfSampleCtx& bc, Sampler& sampler) const {
    auto bxdf = dispatch([&](auto&& x) { return x.sample_bxdf(bc, sampler); });
    bxdf.wi = bc.it.to_local(bc.wi);
    return bxdf;
  }
  vec3 le(const LeEvalCtx& c) const {
    return dispatch([&](auto&& x) { return x.le(c); });
  }
  vec3 albedo(const LeEvalCtx& c) const {
    return dispatch([&]<typename T>(T&& x) {
      if constexpr (psl::same_as<psl::Decay<T>, EmissiveMaterial>) return x.color.eval(c);
      else return x.albedo.eval(c);
    });
  }
};

void material_context(Context& ctx);

}  // namespace pine
