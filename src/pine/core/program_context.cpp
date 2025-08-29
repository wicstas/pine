#include <pine/core/program_context.h>
#include <pine/core/parallel.h>
#include <pine/core/fileio.h>
#include <pine/core/scene.h>
#include <pine/core/rng.h>

#include <pine/impl/integrator/micro_render.h>
#include <pine/impl/integrator/randomwalk.h>
#include <pine/impl/integrator/guidedpath.h>
#include <pine/impl/integrator/cachedpath.h>
#include <pine/impl/integrator/denoiser.h>
#include <pine/impl/integrator/restir.h>
#include <pine/impl/integrator/ears.h>
#include <pine/impl/integrator/path.h>
#include <pine/impl/integrator/mlt.h>
#include <pine/impl/integrator/vol.h>
#include <pine/impl/integrator/ao.h>
#include <pine/impl/accel/embree.h>
#include <pine/impl/accel/bvh.h>

namespace pine {

void setup_program_context() {
  auto &ctx = Context::context;
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
  ctx("print") = [](const psl::string &x) { LOGr(x); };
  ctx("println") = [](const psl::string &x) { LOG(x); };
  ctx("call") = [](psl::function<int()> f) { LOG(f()); };
  ctx("call") = [](psl::function<vec3(vec2)> f) { LOG(f({})); };
  ctx("call") = [](psl::function<mat4()> f) { LOG(f()); };

  ctx.type<BVH>("BVH").ctor();
  ctx.type<EmbreeAccel>("Embree").ctor();
  ctx.type<Accel>("Accel").ctor_variant<EmbreeAccel>();

  ctx.type<UniformLightSampler>("UniformLightSampler").ctor<>();
  ctx.type<LightSampler>("LightSampler").ctor_variant<UniformLightSampler>();

  ctx.type<MicroRenderIntegrator>("MicroRenderGI")
      .ctor<int>()
      .method<&MicroRenderIntegrator::render>("render");

  ctx.type<AOIntegrator>("AOIntegrator")
      .ctor<Accel, Sampler>()
      .method<&AOIntegrator::render>("render");
  ctx("AOIntegrator") = [](Sampler sampler) { return AOIntegrator(EmbreeAccel(), sampler); };


  ctx.type<VolIntegrator>("VolIntegrator")
      .ctor<Accel, Sampler>()
      .method<&VolIntegrator::render>("render");
  ctx("VolIntegrator") = [](Sampler sampler) { return VolIntegrator(EmbreeAccel(), sampler); };

  ctx.type<RandomWalkIntegrator>("RandomWalkIntegrator")
      .ctor<Accel, Sampler, LightSampler, int>()
      .method<&RandomWalkIntegrator::render>("render");
  ctx("RandomWalkIntegrator") = [](Sampler sampler, int max_path_length) {
    return RandomWalkIntegrator(EmbreeAccel(), sampler, UniformLightSampler(), max_path_length);
  };

  ctx.type<PathIntegrator>("PathIntegrator")
      .ctor<Accel, Sampler, LightSampler, int>()
      .method<&PathIntegrator::render>("render");
  ctx("PathIntegrator") = [](Sampler sampler, int max_path_length) {
    return PathIntegrator(EmbreeAccel(), sampler, UniformLightSampler(), max_path_length);
  };

  ctx.type<CachedPathIntegrator>("CachedPathIntegrator")
      .ctor<Accel, Sampler, LightSampler, int, int, int>()
      .method<&CachedPathIntegrator::render>("render");
  ctx("CachedPathIntegrator") = [](Sampler sampler, int max_path_length, int max_axis_resolution) {
    return CachedPathIntegrator(EmbreeAccel(), sampler, UniformLightSampler(), max_path_length,
                                max_axis_resolution, 1);
  };
  ctx("CachedPathIntegrator") = [](Sampler sampler, int max_path_length) {
    return CachedPathIntegrator(EmbreeAccel(), sampler, UniformLightSampler(), max_path_length, 128,
                                1);
  };

  ctx.type<GuidedPathIntegrator>("GuidedPathIntegrator")
      .ctor<Accel, Sampler, LightSampler, int>()
      .method<&GuidedPathIntegrator::render>("render");
  ctx("GuidedPathIntegrator") = [](Sampler sampler, int max_path_length) {
    return GuidedPathIntegrator(EmbreeAccel(), sampler, UniformLightSampler(), max_path_length);
  };

  ctx.type<MltIntegrator>("MltIntegrator")
      .ctor<Accel, int, LightSampler, int>()
      .method<&MltIntegrator::render>("render");
  ctx("MltIntegrator") = [](int spp, int max_path_length) {
    return MltIntegrator(EmbreeAccel(), spp, UniformLightSampler(), max_path_length);
  };

  ctx.type<RestirIntegrator>("RestirIntegrator")
      .ctor<Accel, Sampler, LightSampler, int>()
      .method<&RestirIntegrator::render>("render");
  ctx("RestirIntegrator") = [](Sampler sampler, int max_path_length) {
    return RestirIntegrator(EmbreeAccel(), sampler, UniformLightSampler(), max_path_length);
  };

  ctx("denoise") = [](Scene &scene) {
    DenoiseIntegrator(EmbreeAccel(), SobolSampler(1), LightSampler()).render(scene);
  };

  ctx("quick_render") = [](Scene &scene, vec3 from, vec3 to, psl::string filename) {
    scene.set_camera(ThinLenCamera(Film(vec2i(640, 480)), from, to, 0.5f));
    PathIntegrator(EmbreeAccel(), BlueSobolSampler(4), UniformLightSampler(), 4).render(scene);
    save_film_as_image(filename, scene.camera.film());
  };
}

}  // namespace pine