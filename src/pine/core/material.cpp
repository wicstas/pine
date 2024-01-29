#include <pine/core/scattering.h>
#include <pine/core/material.h>
#include <pine/core/context.h>
#include <pine/core/scene.h>
#include <pine/core/log.h>

namespace pine {

psl::optional<BSDFSample> LayeredMaterial::sample(const MaterialSampleCtx& c) const {
  psl::optional<BSDFSample> bs;
  for (auto&& bsdf : layers) {
    bs = bsdf.sample(c.wi, c.u1, c.u2, c);
    if (bs && SameHemisphere(c.wi, bs->wo))
      break;
  }

  return bs;
}

vec3 LayeredMaterial::F(const MaterialEvalCtx& c) const {
  vec3 f;
  for (auto&& bsdf : layers)
    f += bsdf.f(c.wi, c.wo, c);
  return f;
}
float LayeredMaterial::pdf(const MaterialEvalCtx& c) const {
  float pdf = 0.0f;
  for (auto&& bsdf : layers)
    pdf += bsdf.pdf(c.wi, c.wo, c);
  return pdf;
}

vec3 EmissiveMaterial::le(const LeEvalCtx& ec) const {
  return color.eval(ec);
}

// vec3 Material::BumpNormal(const MaterialEvalCtx& c) const {
// if (!bumpMap)
// return c.n;
// NodeEvalCtx c0 = c, c1 = c;
// const float delta = 0.01f;
// c0.uv += vec2(delta, 0.0f);
// c1.uv += vec2(0.0f, delta);
// c0.p += c.dpdu * delta;
// c1.p += c.dpdv * delta;
// float dddu = (bumpMap->EvalFloat(c0) - bumpMap->EvalFloat(c)) / delta;
// float dddv = (bumpMap->EvalFloat(c1) - bumpMap->EvalFloat(c)) / delta;
// vec3 dpdu = c.dpdu + dddu * c.n;
// vec3 dpdv = c.dpdv + dddv * c.n;
// return face_same_hemisphere(normalize(cross(dpdu, dpdv)), c.n);
// }

void material_context(Context& ctx) {
  ctx.type<DiffuseBSDF>("DiffuseBSDF").ctor<Node3f>();
  ctx.type<ConductorBSDF>("ConductorBSDF").ctor<Node3f, Nodef>();
  ctx.type<DielectricBSDF>("DielectricBSDF").ctor<Node3f, Nodef, Nodef>();
  ctx.type<SpecularReflectionBSDF>("SpecularReflectionBSDF").ctor<Node3f>();
  ctx.type<SpecularRefrectionBSDF>("SpecularRefrectionBSDF").ctor<Node3f, Nodef>();
  ctx.type<BSDF>("BSDF").ctor_variant<DiffuseBSDF, ConductorBSDF, DielectricBSDF>();
  ctx.type<LayeredMaterial>("Layered").ctor<BSDF>().ctor<BSDF, BSDF>();
  ctx.type<DiffuseMaterial>("Diffuse").ctor<Node3f>();
  ctx.type<MetalMaterial>("Metal").ctor<Node3f, Nodef>();
  ctx.type<GlassMaterial>("Glass").ctor<Node3f, Nodef>();
  ctx.type<GlossyMaterial>("Glossy").ctor<Node3f, Node3f, Nodef>();
  ctx.type<MirrorMaterial>("Mirror").ctor<Node3f>();
  ctx.type<WaterMaterial>("Water").ctor<Node3f, Nodef>();
  ctx.type<EmissiveMaterial>("Emissive").ctor<Node3f>();
  ctx.type<Material>("Material")
      .ctor_variant<LayeredMaterial, DiffuseMaterial, MetalMaterial, GlassMaterial, GlossyMaterial,
                    MirrorMaterial, WaterMaterial, EmissiveMaterial>();
}

}  // namespace pine
