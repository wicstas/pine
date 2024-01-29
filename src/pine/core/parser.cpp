#include <pine/core/compiler.h>
#include <pine/core/parser.h>
#include <pine/core/fileio.h>
#include <pine/core/scene.h>
#include <pine/core/rng.h>

#include <pine/impl/integrator/visualizer.h>
#include <pine/impl/integrator/randomwalk.h>
#include <pine/impl/integrator/guidedpath.h>
#include <pine/impl/integrator/path.h>
#include <pine/impl/integrator/ao.h>
#include <pine/impl/accel/bvh.h>

namespace pine {

Context get_default_context() {
  auto ctx = Context{};
  math_context(ctx);
  vecmath_context(ctx);
  array2d_context(ctx);
  rng_context(ctx);
  geometry_context(ctx);
  fileio_context(ctx);
  light_context(ctx);
  node_context(ctx);
  image_context(ctx);
  material_context(ctx);
  film_context(ctx);
  camera_context(ctx);
  scene_context(ctx);
  sampler_context(ctx);
  ctx.type<BVH>("BVH").ctor();
  ctx.type<Accel>("Accel").ctor_variant<BVH>();
  ctx.type<Integrator>("Integrator");
  ctx.type<RTIntegrator>("RTIntegrator");
  ctx.type<PixelIntegrator>("PixelIntegrator");
  ctx.type<RayIntegrator>("RayIntegrator");
  ctx.type<AOIntegrator>("AOIntegrator")
      .ctor<Accel, Sampler>()
      .method("render", &AOIntegrator::render);
  ctx.type<RandomWalkIntegrator>("RandomWalkIntegrator")
      .ctor<Accel, Sampler, int>()
      .method("render", &RandomWalkIntegrator::render);
  ctx.type<PathIntegrator>("PathIntegrator")
      .ctor<Accel, Sampler, LightSampler, int>()
      .method("render", &PathIntegrator::render);
  ctx.type<GuidedPathIntegrator>("GuidedPathIntegrator")
      .ctor<Accel, Sampler, LightSampler, int>()
      .ctor<Accel, Sampler, LightSampler, int, int>()
      .method("render", &GuidedPathIntegrator::render);
  ctx.type<VisualizerIntegrator>("VisIntegrator")
      .ctor<Accel, Sampler, psl::string>()
      .method("render", &VisualizerIntegrator::render);
  ctx.type<UniformLightSampler>("UniformLightSampler").ctor<>();
  ctx.type<LightSampler>("LightSampler").ctor_variant<UniformLightSampler>();
  ctx("print") = +[](psl::span<const Variable*> args) {
    auto ctx = Context();
    ctx("to_string") =
        overloads<bool, int, float, vec2i, vec3i, vec2, vec3, vec4, mat2, mat3, mat4, psl::string>(
            [](const auto& x) {
              using psl::to_string;
              return to_string(x);
            });
    auto str = psl::string();
    for (auto arg : args)
      str += ctx.call("to_string", {&arg, &arg + 1}).as<psl::string>() + ' ';
    if (str.size())
      str.pop_back();
    Logr(str);
  };
  ctx("println") = +[](psl::span<const Variable*> args) {
    auto ctx = Context();
    ctx("to_string") =
        overloads<bool, int, float, vec2i, vec3i, vec2, vec3, vec4, mat2, mat3, mat4, psl::string>(
            [](const auto& x) {
              using psl::to_string;
              return to_string(x);
            });
    auto str = psl::string();
    for (auto arg : args)
      str += ctx.call("to_string", {&arg, &arg + 1}).as<psl::string>() + ' ';
    if (str.size())
      str.pop_back();
    Log(str);
  };

  return ctx;
}

void interpret(Context& context, psl::string source) {
  auto bytecode = compile(context, std::move(source));
  execute(context, bytecode);
}

}  // namespace pine