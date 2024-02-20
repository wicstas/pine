#pragma once
#include <pine/core/interaction.h>
#include <pine/core/scattering.h>
#include <pine/core/bxdf.h>

#include <psl/variant.h>

namespace pine {

struct MaterialSampleCtx : NodeEvalCtx {
  MaterialSampleCtx(vec3 p, vec3 n, vec2 uv, vec3 wi, float u1, vec2 u2)
      : NodeEvalCtx(p, n, uv), m2w(coordinate_system(n)), u1(u1), u2(u2) {
    this->wi = solve(m2w, wi);
  }
  MaterialSampleCtx(const Interaction& it, vec3 wi, float u1, vec2 u2)
      : MaterialSampleCtx(it.p, it.n, it.uv, wi, u1, u2){};

  vec3 wi;
  mat3 m2w;
  float u1;
  vec2 u2;
};

struct MaterialEvalCtx : NodeEvalCtx {
public:
  MaterialEvalCtx(const Interaction& it, vec3 wi, vec3 wo)
      : NodeEvalCtx(it.p, it.n, it.uv), wi(it.to_local(wi)), wo(it.to_local(wo)) {
  }

  vec3 wi;
  vec3 wo;
};

struct LeEvalCtx : NodeEvalCtx {
  LeEvalCtx(vec3 p, vec3 n, vec2 uv, vec3 wo) : NodeEvalCtx(p, n, uv), wo(wo) {
  }
  LeEvalCtx(const Interaction& it, vec3 wo) : NodeEvalCtx(it.p, it.n, it.uv), wo(wo) {
  }

  vec3 wo;
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
  float roughness_amount(const NodeEvalCtx& nc) const {
    return bsdf.roughness_amount(nc);
  }
  bool is_delta() const {
    return false;
  }

private:
  DiffuseBSDF bsdf;
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
  float roughness_amount(const NodeEvalCtx& nc) const {
    return bsdf.roughness_amount(nc);
  }
  bool is_delta() const {
    return false;
  }

private:
  ConductorBSDF bsdf;
};

struct GlassMaterial {
  GlassMaterial(Node3f albedo, Nodef roughness)
      : bsdf{psl::move(albedo), psl::move(roughness), NodeConstant{1.45f}} {
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
  float roughness_amount(const NodeEvalCtx& nc) const {
    return bsdf.roughness_amount(nc);
  }
  bool is_delta() const {
    return false;
  }

private:
  DielectricBSDF bsdf;
};

struct GlossyMaterial {
  GlossyMaterial(Node3f albedo, Node3f specular, Nodef roughness)
      : top{psl::move(specular), psl::move(roughness), NodeConstant{1.45f}},
        bottom{psl::move(albedo)} {
  }

  psl::optional<BSDFSample> sample(const MaterialSampleCtx& mc) const {
    if (auto bs = top.sample(mc.wi, mc.u1, mc.u2, mc); bs && SameHemisphere(mc.wi, bs->wo))
      return bs;
    return bottom.sample(mc.wi, mc.u1, mc.u2, mc);
  }
  vec3 f(const MaterialEvalCtx& mc) const {
    return top.f(mc.wi, mc.wo, mc) + bottom.f(mc.wi, mc.wo, mc);
  }
  float pdf(const MaterialEvalCtx& mc) const {
    return top.pdf(mc.wi, mc.wo, mc) + bottom.pdf(mc.wi, mc.wo, mc);
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
  }
  float roughness_amount(const NodeEvalCtx& nc) const {
    return top.roughness_amount(nc);
  }
  bool is_delta() const {
    return false;
  }

private:
  DielectricBSDF top;
  DiffuseBSDF bottom;
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
  float roughness_amount(const NodeEvalCtx& nc) const {
    return bsdf.roughness_amount(nc);
  }
  bool is_delta() const {
    return true;
  }

private:
  SpecularReflectionBSDF bsdf;
};

struct WaterMaterial {
  WaterMaterial(Node3f albedo, Nodef eta) : bsdf{psl::move(albedo), psl::move(eta)} {
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
  float roughness_amount(const NodeEvalCtx& nc) const {
    return bsdf.roughness_amount(nc);
  }
  bool is_delta() const {
    return true;
  }

private:
  SpecularRefrectionBSDF bsdf;
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
    return color.eval(ec);
  }
  float roughness_amount(const NodeEvalCtx&) const {
    return 1.0f;
  }
  bool is_delta() const {
    return false;
  }

  Node3f color;
};

struct Material : psl::variant<DiffuseMaterial, MetalMaterial, GlassMaterial, GlossyMaterial,
                               MirrorMaterial, WaterMaterial, EmissiveMaterial> {
public:
  using variant::variant;

  psl::optional<BSDFSample> sample(const MaterialSampleCtx& c) const {
    return dispatch([&](auto&& x) {
      psl::optional<BSDFSample> bs = x.sample(c);
      if (bs)
        bs->wo = c.m2w * bs->wo;
      return bs;
    });
  }
  vec3 f(const MaterialEvalCtx& c) const {
    return dispatch([&](auto&& x) { return x.f(c); });
  }
  float pdf(const MaterialEvalCtx& c) const {
    return dispatch([&](auto&& x) { return x.pdf(c); });
  }
  psl::pair<vec3, float> f_pdf(const MaterialEvalCtx& c) const {
    return {f(c), pdf(c)};
  }
  vec3 le(const LeEvalCtx& c) const {
    return dispatch([&](auto&& x) { return x.le(c); });
  }
  float roughness_amount(const NodeEvalCtx& nc) const {
    return dispatch([&](auto&& x) { return x.roughness_amount(nc); });
  }
  bool is_delta() const {
    return dispatch([&](auto&& x) { return x.is_delta(); });
  }
};

void material_context(Context& ctx);

}  // namespace pine
