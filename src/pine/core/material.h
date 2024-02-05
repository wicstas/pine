#pragma once

#include <pine/core/interaction.h>
#include <pine/core/scattering.h>
#include <pine/core/bxdf.h>

#include <psl/variant.h>

namespace pine {

struct MaterialSampleCtx : NodeEvalCtx {
  MaterialSampleCtx(vec3 p, vec3 n, vec2 uv, vec3 wi, float u1, vec2 u2)
      : NodeEvalCtx(p, n, uv), m2w(coordinate_system(n)), u1(u1), u2(u2) {
    auto w2m = inverse(m2w);
    this->wi = w2m * wi;
  };
  MaterialSampleCtx(const Interaction& it, vec3 wi, float u1, vec2 u2)
      : MaterialSampleCtx(it.p, it.n, it.uv, wi, u1, u2){};

  vec3 wi;
  mat3 m2w;
  float u1;
  vec2 u2;
};

struct MaterialEvalCtx : NodeEvalCtx {
  MaterialEvalCtx(vec3 p, vec3 n, vec2 uv, vec3 wi, vec3 wo) : NodeEvalCtx(p, n, uv) {
    auto w2m = inverse(coordinate_system(n));
    this->wi = w2m * wi;
    this->wo = w2m * wo;
  };
  MaterialEvalCtx(const Interaction& it, vec3 wi, vec3 wo)
      : MaterialEvalCtx(it.p, it.n, it.uv, wi, wo){};

  vec3 wi;
  vec3 wo;
};

struct LeEvalCtx : NodeEvalCtx {
  LeEvalCtx(vec3 p, vec3 n, vec2 uv, vec3 wo) : NodeEvalCtx(p, n, uv) {
    auto w2m = inverse(coordinate_system(n));
    this->wo = w2m * wo;
  };
  LeEvalCtx(const Interaction& it, vec3 wo) : LeEvalCtx(it.p, it.n, it.uv, wo){};

  vec3 wo;
};

struct LayeredMaterial {
  template <psl::SameAs<BSDF>... Ts>
  LayeredMaterial(Ts... layers) : layers(psl::vector_of(psl::move(layers)...)) {
  }
  LayeredMaterial(psl::vector<BSDF> layers) : layers(psl::move(layers)) {
  }

  psl::optional<BSDFSample> sample(const MaterialSampleCtx& c) const;
  vec3 F(const MaterialEvalCtx& c) const;
  float pdf(const MaterialEvalCtx& c) const;
  vec3 le(const LeEvalCtx&) const {
    return {};
  }
  bool is_delta() const {
    for (auto& layer : layers)
      if (!layer.is_delta())
        return false;
    return true;
  }

  psl::vector<BSDF> layers;
};

struct DiffuseMaterial {
  DiffuseMaterial(Node3f albedo) : bsdf{psl::move(albedo)} {
  }

  psl::optional<BSDFSample> sample(const MaterialSampleCtx& mc) const {
    return bsdf.sample(mc.wi, mc.u1, mc.u2, mc);
  }
  vec3 F(const MaterialEvalCtx& mc) const {
    return bsdf.f(mc.wi, mc.wo, mc);
  }
  float pdf(const MaterialEvalCtx& mc) const {
    return bsdf.pdf(mc.wi, mc.wo, mc);
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
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
  vec3 F(const MaterialEvalCtx& mc) const {
    return bsdf.f(mc.wi, mc.wo, mc);
  }
  float pdf(const MaterialEvalCtx& mc) const {
    return bsdf.pdf(mc.wi, mc.wo, mc);
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
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
  vec3 F(const MaterialEvalCtx& mc) const {
    return bsdf.f(mc.wi, mc.wo, mc);
  }
  float pdf(const MaterialEvalCtx& mc) const {
    return bsdf.pdf(mc.wi, mc.wo, mc);
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
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
  vec3 F(const MaterialEvalCtx& mc) const {
    return top.f(mc.wi, mc.wo, mc) + bottom.f(mc.wi, mc.wo, mc);
  }
  float pdf(const MaterialEvalCtx& mc) const {
    return top.pdf(mc.wi, mc.wo, mc) + bottom.pdf(mc.wi, mc.wo, mc);
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
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
  vec3 F(const MaterialEvalCtx& mc) const {
    return bsdf.f(mc.wi, mc.wo, mc);
  }
  float pdf(const MaterialEvalCtx& mc) const {
    return bsdf.pdf(mc.wi, mc.wo, mc);
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
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
  vec3 F(const MaterialEvalCtx& mc) const {
    return bsdf.f(mc.wi, mc.wo, mc);
  }
  float pdf(const MaterialEvalCtx& mc) const {
    return bsdf.pdf(mc.wi, mc.wo, mc);
  }
  vec3 le(const LeEvalCtx&) const {
    return {};
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
  vec3 F(const MaterialEvalCtx&) const {
    return {};
  }
  float pdf(const MaterialEvalCtx&) const {
    return {};
  }
  vec3 le(const LeEvalCtx&) const;
  bool is_delta() const {
    return false;
  }

  Node3f color;
};

struct Material : psl::variant<LayeredMaterial, DiffuseMaterial, MetalMaterial, GlassMaterial,
                               GlossyMaterial, MirrorMaterial, WaterMaterial, EmissiveMaterial> {
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
  vec3 F(const MaterialEvalCtx& c) const {
    return dispatch([&](auto&& x) { return x.F(c); });
  }
  float pdf(const MaterialEvalCtx& c) const {
    return dispatch([&](auto&& x) { return x.pdf(c); });
  }
  vec3 le(const LeEvalCtx& c) const {
    return dispatch([&](auto&& x) { return x.le(c); });
  }
  bool is_delta() const {
    return dispatch([&](auto&& x) { return x.is_delta(); });
  }
};

void material_context(Context& ctx);

}  // namespace pine
