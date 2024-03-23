#pragma once
#include <pine/core/interaction.h>
#include <pine/core/scattering.h>
#include <pine/core/bxdf.h>

#include <psl/variant.h>

namespace pine {

struct MaterialSampleCtx : BxdfEvalCtx {
  MaterialSampleCtx(const SurfaceInteraction& it, vec3 wi, float u1, vec2 u2,
                    float min_roughness = 0.0f)
      : BxdfEvalCtx(it, min_roughness), m2w(coordinate_system(it.n)), u1(u1), u2(u2) {
    this->wi = solve(m2w, wi);
  };

  vec3 wi;
  mat3 m2w;
  float u1;
  vec2 u2;
};

struct MaterialEvalCtx : BxdfEvalCtx {
public:
  MaterialEvalCtx(const SurfaceInteraction& it, vec3 wi, vec3 wo, float min_roughness = 0.0f)
      : BxdfEvalCtx(it, min_roughness), wi(it.to_local(wi)), wo(it.to_local(wo)) {
  }

  vec3 wi;
  vec3 wo;
};

struct LeEvalCtx : NodeEvalCtx {
  LeEvalCtx(vec3 p, vec3 n, vec2 uv, vec3 wo) : NodeEvalCtx(p, n, uv), wo(wo) {
  }
  LeEvalCtx(const SurfaceInteraction& it, vec3 wo) : NodeEvalCtx(it.p, it.n, it.uv), wo(wo) {
  }

  vec3 wo;
};

struct EmissiveMaterial {
  EmissiveMaterial(Node3f color) : color{psl::move(color)} {
  }

  psl::optional<BSDFSample> sample(const MaterialSampleCtx&) const {
    return psl::nullopt;
  }
  vec3 f(const MaterialEvalCtx&) const {
    return {};
  }
  float pdf(const MaterialEvalCtx&) const {
    return {};
  }
  vec3 le(const LeEvalCtx& ec) const {
    if (dot(ec.wo, ec.n) < 0.0f)
      return vec3(0.0f);
    return color.eval(ec);
  }
  float roughness_amount(const BxdfEvalCtx&) const {
    return 1.0f;
  }
  vec3 albedo(const BxdfEvalCtx& bc) const {
    return color(bc);
  }

  Node3f color;
};

struct DiffuseMaterial {
  DiffuseMaterial(Node3f albedo) : bsdf{psl::move(albedo)} {
  }

  psl::optional<BSDFSample> sample(const MaterialSampleCtx& mc) const {
    return bsdf.sample(mc.wi, mc.u1, mc.u2, mc);
  }
  vec3 f(const MaterialEvalCtx& mc) const {
    return bsdf.f(mc.wi, mc.wo, mc);
  }
  float pdf(const MaterialEvalCtx& mc) const {
    return bsdf.pdf(mc.wi, mc.wo, mc);
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
  }
  float roughness_amount(const BxdfEvalCtx& bc) const {
    return bsdf.roughness_amount(bc);
  }
  vec3 albedo(const BxdfEvalCtx& bc) const {
    return bsdf.albedo(bc);
  }

  DiffuseBSDF bsdf;
};

struct MirrorMaterial {
  MirrorMaterial(Node3f albedo) : bsdf{psl::move(albedo)} {
  }

  psl::optional<BSDFSample> sample(const MaterialSampleCtx& mc) const {
    return bsdf.sample(mc.wi, mc.u1, mc.u2, mc);
  }
  vec3 f(const MaterialEvalCtx& mc) const {
    return bsdf.f(mc.wi, mc.wo, mc);
  }
  float pdf(const MaterialEvalCtx& mc) const {
    return bsdf.pdf(mc.wi, mc.wo, mc);
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
  }
  float roughness_amount(const BxdfEvalCtx& bc) const {
    return bsdf.roughness_amount(bc);
  }
  vec3 albedo(const BxdfEvalCtx& bc) const {
    return bsdf.albedo(bc);
  }

private:
  SpecularReflectionBSDF bsdf;
};

struct MetalMaterial {
  MetalMaterial(Node3f albedo, Nodef roughness) : bsdf{psl::move(albedo), psl::move(roughness)} {
  }

  psl::optional<BSDFSample> sample(const MaterialSampleCtx& mc) const {
    return bsdf.sample(mc.wi, mc.u1, mc.u2, mc);
  }
  vec3 f(const MaterialEvalCtx& mc) const {
    return bsdf.f(mc.wi, mc.wo, mc);
  }
  float pdf(const MaterialEvalCtx& mc) const {
    return bsdf.pdf(mc.wi, mc.wo, mc);
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
  }
  float roughness_amount(const BxdfEvalCtx& bc) const {
    return bsdf.roughness_amount(bc);
  }
  vec3 albedo(const BxdfEvalCtx& bc) const {
    return bsdf.albedo(bc);
  }

  ConductorBSDF bsdf;
};

struct GlossyMaterial {
  GlossyMaterial(Node3f albedo, Nodef roughness, Nodef eta)
      : bsdf{psl::move(albedo), psl::move(roughness), psl::move(eta)} {
  }

  psl::optional<BSDFSample> sample(const MaterialSampleCtx& mc) const {
    return bsdf.sample(mc.wi, mc.u1, mc.u2, mc);
  }
  vec3 f(const MaterialEvalCtx& mc) const {
    return bsdf.f(mc.wi, mc.wo, mc);
  }
  float pdf(const MaterialEvalCtx& mc) const {
    return bsdf.pdf(mc.wi, mc.wo, mc);
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
  }
  float roughness_amount(const BxdfEvalCtx& bc) const {
    return bsdf.roughness_amount(bc);
  }
  vec3 albedo(const BxdfEvalCtx& bc) const {
    return bsdf.albedo(bc);
  }

  DiffusiveDielectricBSDF bsdf;
};

struct GlassMaterial {
  GlassMaterial(Node3f albedo, Nodef roughness, Nodef eta)
      : bsdf{psl::move(albedo), psl::move(roughness), psl::move(eta)} {
  }

  psl::optional<BSDFSample> sample(const MaterialSampleCtx& mc) const {
    return bsdf.sample(mc.wi, mc.u1, mc.u2, mc);
  }
  vec3 f(const MaterialEvalCtx& mc) const {
    return bsdf.f(mc.wi, mc.wo, mc);
  }
  float pdf(const MaterialEvalCtx& mc) const {
    return bsdf.pdf(mc.wi, mc.wo, mc);
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
  }
  float roughness_amount(const BxdfEvalCtx& bc) const {
    return bsdf.roughness_amount(bc);
  }
  vec3 albedo(const BxdfEvalCtx& bc) const {
    return bsdf.albedo(bc);
  }

  RefractiveDielectricBSDF bsdf;
};

template <typename A, typename B>
struct BlendMaterial {
  BlendMaterial(Nodef factor, A a, B b)
      : factor(psl::move(factor)), a(psl::move(a)), b(psl::move(b)) {
  }

  psl::optional<BSDFSample> sample(const MaterialSampleCtx& mc_) const {
    auto mc = mc_;
    if (with_prob(factor(mc), mc.u1))
      return b.sample(mc);
    else
      return a.sample(mc);
  }
  vec3 f(const MaterialEvalCtx& mc) const {
    auto r = factor(mc);
    if (r == 0.0f)
      return a.f(mc);
    else if (r == 1.0f)
      return b.f(mc);
    else
      return lerp(r, a.f(mc), b.f(mc));
  }
  float pdf(const MaterialEvalCtx& mc) const {
    auto r = factor(mc);
    if (r == 0.0f)
      return a.pdf(mc);
    else if (r == 1.0f)
      return b.pdf(mc);
    else
      return lerp(r, a.pdf(mc), b.pdf(mc));
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
  }
  float roughness_amount(const BxdfEvalCtx& bc) const {
    return lerp(factor(bc), a.roughness_amount(bc), b.roughness_amount(bc));
  }
  vec3 albedo(const BxdfEvalCtx& bc) const {
    return a.albedo(bc);
  }

private:
  Nodef factor;
  A a;
  B b;
};

struct UberMaterial {
  using M0 = BlendMaterial<GlossyMaterial, GlassMaterial>;

  UberMaterial(Node3f albedo, Nodef roughness, Nodef metallic = 0.0f, Nodef transmission = 0.0f,
               Nodef ior = 1.3f)
      : base(metallic,
             M0(transmission, GlossyMaterial(albedo, roughness, ior),
                GlassMaterial(albedo, roughness, ior)),
             MetalMaterial(albedo, roughness)) {
  }

  psl::optional<BSDFSample> sample(const MaterialSampleCtx& mc) const {
    return base.sample(mc);
  }
  vec3 f(const MaterialEvalCtx& mc) const {
    return base.f(mc);
  }
  float pdf(const MaterialEvalCtx& mc) const {
    return base.pdf(mc);
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
  }
  float roughness_amount(const BxdfEvalCtx& bc) const {
    return base.roughness_amount(bc);
  }
  vec3 albedo(const BxdfEvalCtx& bc) const {
    return base.albedo(bc);
  }

private:
  BlendMaterial<M0, MetalMaterial> base;
};

struct SubsurfaceMaterial {
  SubsurfaceMaterial(Node3f albedo, Nodef roughness, vec3 sigma_s)
      : base{albedo, roughness, 1.3f}, diffuse(albedo), sigma_s(sigma_s) {
  }

  psl::optional<BSDFSample> sample(const MaterialSampleCtx& mc) const {
    auto s = base.sample(mc);
    if (s)
      s->enter_subsurface = CosTheta(s->wo) < 0.0f;
    return s;
  }
  vec3 f(MaterialEvalCtx mc) const {
    if (!SameHemisphere(mc.wi, mc.wo)) {
      if (CosTheta(mc.wi) < 0)
        mc.wi = -mc.wi;
      else
        mc.wo = -mc.wo;
      return diffuse.f(mc);
    } else {
      return base.f(mc);
    }
  }
  float pdf(MaterialEvalCtx mc) const {
    if (!SameHemisphere(mc.wi, mc.wo)) {
      if (CosTheta(mc.wi) < 0)
        mc.wi = -mc.wi;
      else
        mc.wo = -mc.wo;
      return diffuse.pdf(mc);
    } else {
      return base.pdf(mc);
    }
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
  }
  float roughness_amount(const BxdfEvalCtx& bc) const {
    return base.roughness_amount(bc);
  }
  vec3 albedo(const BxdfEvalCtx& bc) const {
    return base.albedo(bc);
  }
  GlassMaterial base;
  DiffuseMaterial diffuse;
  vec3 sigma_s;
};

struct Material : psl::variant<EmissiveMaterial, DiffuseMaterial, MirrorMaterial, UberMaterial,
                               SubsurfaceMaterial> {
public:
  using variant::variant;

  psl::optional<BSDFSample> sample(const MaterialSampleCtx& c) const {
    if (c.wi.z == 0.0f)
      return psl::nullopt;
    return dispatch([&](auto&& x) {
      psl::optional<BSDFSample> bs = x.sample(c);
      if (bs) {
        bs->wo = c.m2w * bs->wo;
        if (bs->pdf == 0.0f)
          bs = psl::nullopt;
      }
      return bs;
    });
  }
  vec3 f(const MaterialEvalCtx& c) const {
    if (c.wi.z * c.wo.z == 0.0f)
      return vec3(0.0f);
    return dispatch([&](auto&& x) { return x.f(c); });
  }
  float pdf(const MaterialEvalCtx& c) const {
    if (c.wi.z * c.wo.z == 0.0f)
      return 0.0f;
    return dispatch([&](auto&& x) { return x.pdf(c); });
  }
  psl::pair<vec3, float> f_pdf(const MaterialEvalCtx& c) const {
    return {f(c), pdf(c)};
  }
  vec3 le(const LeEvalCtx& c) const {
    return dispatch([&](auto&& x) { return x.le(c); });
  }
  vec3 albedo(const BxdfEvalCtx& bc) const {
    return dispatch([&](auto&& x) { return x.albedo(bc); });
  }
  float roughness_amount(const BxdfEvalCtx& bc) const {
    return dispatch([&](auto&& x) { return x.roughness_amount(bc); });
  }
  bool is_delta(const BxdfEvalCtx& bc) const {
    return roughness_amount(bc) < 0.005f;
  }
};

void material_context(Context& ctx);

}  // namespace pine
