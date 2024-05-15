#include <pine/core/program_context.h>
#include <pine/core/compiler.h>
#include <pine/core/parallel.h>
#include <pine/core/fileio.h>
#include <pine/core/scene.h>
#include <pine/core/rng.h>

#include <pine/impl/integrator/randomwalk.h>
#include <pine/impl/integrator/guidedpath.h>
#include <pine/impl/integrator/cachedpath.h>
#include <pine/impl/integrator/denoiser.h>
#include <pine/impl/integrator/restir.h>
#include <pine/impl/integrator/ears.h>
#include <pine/impl/integrator/path.h>
#include <pine/impl/integrator/ao.h>
#include <pine/impl/accel/embree.h>
#include <pine/impl/accel/bvh.h>

namespace pine {

Context get_program_context() {
  auto ctx = Context{};
  math_context(ctx);
  vecmath_context(ctx);
  array2d_context(ctx);
  rng_context(ctx);
  image_context(ctx);
  node_context(ctx);
  material_context(ctx);
  geometry_context(ctx);
  medium_context(ctx);
  light_context(ctx);
  film_context(ctx);
  camera_context(ctx);
  scene_context(ctx);
  sampler_context(ctx);
  fileio_context(ctx);
  parallel_context(ctx);
  ctx("print") = +[](const psl::string &x) { Logr(x); };
  ctx("println") = +[](const psl::string &x) { Log(x); };
  ctx.type<BVH>("BVH").ctor();
  ctx.type<EmbreeAccel>("Embree").ctor();
  ctx.type<Accel>("Accel").ctor_variant<EmbreeAccel>();
  ctx.type<UniformLightSampler>("UniformLightSampler").ctor<>();
  ctx.type<LightSampler>("LightSampler").ctor_variant<UniformLightSampler>();
  ctx.type<RayIntegrator>("RayIntegrator")
      .method("intersect", &RayIntegrator::intersect)
      .method("hit", &RayIntegrator::hit);
  ctx.type<CustomRayIntegrator>("CustomRayIntegrator")
      .ctor(+[](Sampler sampler, psl::function<vec3(RayIntegrator &, Ray, Sampler &)> f) {
        return CustomRayIntegrator(EmbreeAccel(), sampler, MOVE(f));
      })
      .method("render", &CustomRayIntegrator::render);
  ctx.type<AOIntegrator>("AOIntegrator")
      .ctor<Accel, Sampler>()
      .ctor(+[](Sampler sampler) { return AOIntegrator(EmbreeAccel(), sampler); })
      .method("render", &AOIntegrator::render);
  ctx.type<RandomWalkIntegrator>("RandomWalkIntegrator")
      .ctor<Accel, Sampler, int>()
      .ctor(+[](Sampler sampler, int max_path_length) {
        return RandomWalkIntegrator(EmbreeAccel(), sampler, max_path_length);
      })
      .method("render", &RandomWalkIntegrator::render);
  ctx.type<PathIntegrator>("PathIntegrator")
      .ctor<Accel, Sampler, LightSampler, int>()
      .ctor(+[](Sampler sampler, int max_path_length) {
        return PathIntegrator(EmbreeAccel(), sampler, UniformLightSampler(), max_path_length);
      })
      .method("render", &PathIntegrator::render);
  ctx.type<CachedPathIntegrator>("CachedPathIntegrator")
      .ctor<Accel, Sampler, LightSampler, int, int, int>()
      .ctor(+[](Sampler sampler, int max_path_length, int max_axis_resolution) {
        return CachedPathIntegrator(EmbreeAccel(), sampler, UniformLightSampler(), max_path_length,
                                    max_axis_resolution, 1);
      })
      .ctor(+[](Sampler sampler, int max_path_length) {
        return CachedPathIntegrator(EmbreeAccel(), sampler, UniformLightSampler(), max_path_length,
                                    128, 1);
      })
      .method("render", &CachedPathIntegrator::render);
  ctx.type<GuidedPathIntegrator>("GuidedPathIntegrator")
      .ctor<Accel, Sampler, LightSampler, int>()
      .ctor(+[](Sampler sampler, int max_path_length) {
        return GuidedPathIntegrator(EmbreeAccel(), sampler, UniformLightSampler(), max_path_length);
      })
      .method("render", &GuidedPathIntegrator::render);
  ctx.type<RestirIntegrator>("RestirIntegrator")
      .ctor<Accel, Sampler, LightSampler, int>()
      .ctor(+[](Sampler sampler, int max_path_length) {
        return RestirIntegrator(EmbreeAccel(), sampler, UniformLightSampler(), max_path_length);
      })
      .method("render", &RestirIntegrator::render);
  ctx("denoise") =
      +[](Scene &scene) { DenoiseIntegrator(EmbreeAccel(), SobolSampler(1)).render(scene); };

  return ctx;
}

}  // namespace pine