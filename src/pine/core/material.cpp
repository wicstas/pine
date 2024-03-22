#include <pine/core/scattering.h>
#include <pine/core/material.h>
#include <pine/core/context.h>
#include <pine/core/scene.h>
#include <pine/core/log.h>

namespace pine {

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
  // ctx.type<DiffuseBSDF>("DiffuseBSDF").ctor<Node3f>();
  // ctx.type<ConductorBSDF>("ConductorBSDF").ctor<Node3f, Nodef>();
  // ctx.type<SpecularReflectionBSDF>("SpecularReflectionBSDF").ctor<Node3f>();
  // ctx.type<SpecularRefrectionBSDF>("SpecularRefrectionBSDF").ctor<Node3f, Nodef>();
  // ctx.type<BSDF>("BSDF").ctor_variant<DiffuseBSDF, ConductorBSDF, DielectricBSDF>();
  ctx.type<EmissiveMaterial>("Emissive").ctor<Node3f>();
  ctx.type<DiffuseMaterial>("Diffuse").ctor<Node3f>();
  ctx.type<MirrorMaterial>("Mirror").ctor<Node3f>();
  ctx.type<SubsurfaceMaterial>("Subsurface").ctor<Node3f, Nodef, vec3>();
  ctx.type<UberMaterial>("Uber")
      .ctor<Node3f, Nodef>()
      .ctor<Node3f, Nodef, Nodef>()
      .ctor<Node3f, Nodef, Nodef, Nodef>()
      .ctor<Node3f, Nodef, Nodef, Nodef, Nodef>();
  ctx.type<Material>("Material")
      .ctor_variant<EmissiveMaterial, DiffuseMaterial, MirrorMaterial, UberMaterial,
                    SubsurfaceMaterial>();
}

}  // namespace pine
