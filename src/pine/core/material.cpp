#include <pine/core/scattering.h>
#include <pine/core/material.h>
#include <pine/core/context.h>
#include <pine/core/scene.h>
#include <pine/core/log.h>

namespace pine {

BXDF UberMaterial::sample_bxdf(const BxdfSampleCtx& bc, Sampler& sampler) const {
  if (with_probability(metallic(bc), sampler)) {
    return ConductorBSDF(albedo(bc), roughness(bc));
  } else {
    if (with_probability(transmission(bc), sampler))
      return RefractiveDielectricBSDF(albedo(bc), roughness(bc), ior);
    else
      return DiffusiveDielectricBSDF(albedo(bc), roughness(bc), ior);
  }
}

BXDF SubsurfaceMaterial::sample_bxdf(const BxdfSampleCtx& bc, Sampler& sampler) const {
  auto fr = FrDielectric(dot(bc.wi, bc.n), ior);
  if (sampler.get1d() < fr)
    return RefractiveBSDF(albedo(bc), psl::max(roughness(bc), bc.min_roughness), ior);
  else if (bc.diffused)
    return DiffuseBSDF(albedo(bc));
  else
    return BSSRDF(albedo(bc), ior, sigma_s);
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
  ctx.type<EmissiveMaterial>("Emissive").ctor<Node3f>();
  ctx.type<DiffuseMaterial>("Diffuse").ctor<Node3f>();
  ctx.type<MetalMaterial>("Metal").ctor<Node3f, Nodef>();
  ctx.type<GlossyMaterial>("Glossy").ctor<Node3f, Nodef>().ctor<Node3f, Nodef, Nodef>();
  ctx.type<GlassMaterial>("Glass").ctor<Node3f, Nodef>().ctor<Node3f, Nodef, Nodef>();
  ctx.type<SubsurfaceMaterial>("Subsurface").ctor<Node3f, Nodef, vec3>();
  ctx.type<UberMaterial>("Uber")
      .ctor<Node3f, Nodef>()
      .ctor<Node3f, Nodef, Nodef>()
      .ctor<Node3f, Nodef, Nodef, Nodef>()
      .ctor<Node3f, Nodef, Nodef, Nodef, float>();
  ctx.type<Material>("Material")
      .ctor_variant<EmissiveMaterial, DiffuseMaterial, MetalMaterial, GlossyMaterial, GlassMaterial,
                    UberMaterial, SubsurfaceMaterial>();
  ctx.type<psl::shared_ptr<Material>>("MaterialPtr");
}

}  // namespace pine
