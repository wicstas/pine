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
  ctx.type<PointLight>("PointLight").ctor<vec3, vec3>();
  ctx.type<DirectionalLight>("DirectionalLight").ctor<vec3, vec3>();
  ctx.type<Light>("Light").ctor_variant<PointLight, DirectionalLight>();
  ctx.type<Sky>("Sky").ctor<vec3>();
  ctx.type<Atmosphere>("Atmosphere").ctor<vec3, vec3>();
  ctx.type<ImageSky>("ImageSky")
      .ctor<psl::shared_ptr<Image>>()
      .ctor<psl::shared_ptr<Image>, vec3>();
  ctx.type<EnvironmentLight>("EnvironmentLight").ctor_variant<Atmosphere, Sky, ImageSky>();
  ctx.type<NodePosition>("Position").ctor<>();
  ctx.type<NodeNormal>("Normal").ctor<>();
  ctx.type<NodeUV>("UV").ctor<>();
  ctx.type<NodeConstant<float>>("Constantf").ctor_variant<float>();
  ctx.type<NodeConstant<vec3>>("Constant3f").ctor_variant<vec3>();
  ctx.type<NodeBinary<float, '+'>>("Addf").ctor<Nodef, Nodef>();
  ctx.type<NodeBinary<vec3, '+'>>("Add3f").ctor<Node3f, Node3f>();
  ctx.type<NodeBinary<float, '-'>>("Subf").ctor<Nodef, Nodef>();
  ctx.type<NodeBinary<vec3, '-'>>("Sub3f").ctor<Node3f, Node3f>();
  ctx.type<NodeBinary<float, '*'>>("Mulf").ctor<Nodef, Nodef>();
  ctx.type<NodeBinary<vec3, '*'>>("Mul3f").ctor<Node3f, Node3f>();
  ctx.type<NodeBinary<float, '/'>>("Divf").ctor<Nodef, Nodef>();
  ctx.type<NodeBinary<vec3, '/'>>("Div3f").ctor<Node3f, Node3f>();
  ctx.type<NodeBinary<float, '^'>>("Powf").ctor<Nodef, Nodef>();
  ctx.type<NodeBinary<vec3, '^'>>("Pow3f").ctor<Node3f, Node3f>();
  ctx.type<NodeUnary<float, '-'>>("Negf").ctor<Nodef>();
  ctx.type<NodeUnary<vec3, '-'>>("Neg3f").ctor<Node3f>();
  ctx.type<NodeUnary<float, 'a'>>("Absf").ctor<Nodef>();
  ctx.type<NodeUnary<vec3, 'a'>>("Abs3f").ctor<Node3f>();
  ctx.type<NodeUnary<float, 's'>>("Sqrf").ctor<Nodef>();
  ctx.type<NodeUnary<vec3, 's'>>("Sqr3f").ctor<Node3f>();
  ctx.type<NodeUnary<float, 'r'>>("Sqrtf").ctor<Nodef>();
  ctx.type<NodeUnary<vec3, 'r'>>("Sqrt3f").ctor<Node3f>();
  ctx.type<NodeUnary<float, 'f'>>("Fractf").ctor<Nodef>();
  ctx.type<NodeUnary<vec3, 'f'>>("Fract3f").ctor<Node3f>();
  ctx.type<NodeComponent>("Comp").ctor<Node3f, int>();
  ctx.type<NodeToVec3>("Vec3").ctor<Nodef>().ctor<Nodef, Nodef, Nodef>();
  ctx.type<NodeNoisef>("Noisef").ctor<Node3f, Nodef, Nodef>();
  ctx.type<NodeNoise3f>("Noise3f").ctor<Node3f, Nodef, Nodef>();
  ctx.type<NodeCheckerboard>("Checkerboard").ctor<Node3f>().ctor<Node3f, float>();
  ctx.type<psl::shared_ptr<Image>>("Image");
  ctx.type<NodeImage>("Texture").ctor<Node3f, psl::shared_ptr<Image>>();
  ctx("load_image") =
      +[](psl::string filepath) { return psl::make_shared<Image>(read_image(filepath)); };
  ctx("+") = +[](Nodef a, Nodef b) { return NodeBinary<float, '+'>{psl::move(a), psl::move(b)}; };
  ctx("-") = +[](Nodef a, Nodef b) { return NodeBinary<float, '-'>{psl::move(a), psl::move(b)}; };
  ctx("*") = +[](Nodef a, Nodef b) { return NodeBinary<float, '*'>{psl::move(a), psl::move(b)}; };
  ctx("/") = +[](Nodef a, Nodef b) { return NodeBinary<float, '/'>{psl::move(a), psl::move(b)}; };
  ctx("^") = +[](Nodef a, Nodef b) { return NodeBinary<float, '^'>{psl::move(a), psl::move(b)}; };
  ctx("-x") = +[](Nodef a) { return NodeUnary<float, '-'>{psl::move(a)}; };
  ctx("+") = +[](Node3f a, Node3f b) { return NodeBinary<vec3, '+'>{psl::move(a), psl::move(b)}; };
  ctx("-") = +[](Node3f a, Node3f b) { return NodeBinary<vec3, '-'>{psl::move(a), psl::move(b)}; };
  ctx("*") = +[](Node3f a, Node3f b) { return NodeBinary<vec3, '*'>{psl::move(a), psl::move(b)}; };
  ctx("/") = +[](Node3f a, Node3f b) { return NodeBinary<vec3, '/'>{psl::move(a), psl::move(b)}; };
  ctx("^") = +[](Node3f a, Node3f b) { return NodeBinary<vec3, '^'>{psl::move(a), psl::move(b)}; };
  ctx("-x") = +[](Node3f a) { return NodeUnary<vec3, '-'>{psl::move(a)}; };
  ctx("*") = +[](Node3f a, Nodef b) { return NodeBinary<vec3, '*'>{psl::move(a), psl::move(b)}; };
  ctx("*") = +[](Nodef a, Node3f b) { return NodeBinary<vec3, '*'>{psl::move(a), psl::move(b)}; };
  ctx("^") = +[](Node3f a, Nodef b) { return NodeBinary<vec3, '^'>{psl::move(a), psl::move(b)}; };
  ctx("/") = +[](Node3f a, Nodef b) { return NodeBinary<vec3, '/'>{psl::move(a), psl::move(b)}; };
  ctx("/") = +[](Nodef a, Node3f b) { return NodeBinary<vec3, '/'>{psl::move(a), psl::move(b)}; };
  ctx("[]") = +[](Node3f a, int index) { return NodeComponent{psl::move(a), index}; };
  ctx.type<Nodef>("Nodef")
      .ctor_variant<int, float, NodeConstant<float>, NodeBinary<float, '+'>, NodeBinary<float, '-'>,
                    NodeBinary<float, '*'>, NodeBinary<float, '/'>, NodeBinary<float, '^'>,
                    NodeUnary<float, '-'>, NodeUnary<float, 'a'>, NodeUnary<float, 's'>,
                    NodeUnary<float, 'r'>, NodeUnary<float, 'f'>, NodeComponent, NodeNoisef,
                    NodeCheckerboard>();
  ctx.type<Node3f>("Node3f")
      .ctor_variant<vec3i, vec3, NodeConstant<vec3>, NodeBinary<vec3, '+'>, NodeBinary<vec3, '-'>,
                    NodeBinary<vec3, '*'>, NodeBinary<vec3, '/'>, NodeBinary<vec3, '^'>,
                    NodeUnary<vec3, '-'>, NodeUnary<vec3, 'a'>, NodeUnary<vec3, 's'>,
                    NodeUnary<vec3, 'r'>, NodeUnary<vec3, 'f'>, NodeToVec3, NodeNoise3f,
                    NodePosition, NodeNormal, NodeUV, NodeImage>();
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
  ctx.type<Film>("Film")
      .ctor<vec2i>()
      .method("scale", &Film::scale)
      .method("offset", &Film::offset);
  ctx.type<ThinLenCamera>("ThinLenCamera")
      .ctor<Film, vec3, vec3, float>()
      .ctor<Film, vec3, vec3, float, float, float>();
  ctx.type<Camera>("Camera").ctor_variant<ThinLenCamera>().method("film", &Camera::film);
  ctx.type<Scene>("Scene")
      .ctor<>()
      .member("camera", &Scene::camera)
      .method("add", &Scene::add_material)
      .method("add", overloaded<Shape, psl::string>(&Scene::add_geometry))
      .method("add", overloaded<Shape, Material>(&Scene::add_geometry))
      .method("add", overloaded<Light>(&Scene::add_light))
      .method("set", &Scene::set_camera)
      .method("set", &Scene::set_env_light)
      .method("reset", &Scene::reset);
  ctx("add_box") = overloaded<Scene&, mat4, Material>(add_box);
  ctx("add_box") = overloaded<Scene&, mat4, psl::string>(add_box);
  ctx.type<UniformSampler>("UniformSampler").ctor<int>();
  ctx.type<HaltonSampler>("HaltonSampler").ctor<int, vec2i>();
  ctx.type<Sampler>("Sampler").ctor_variant<UniformSampler, HaltonSampler>();
  ctx.type<BVH>("BVH").ctor();
  ctx.type<Accel>("Accel").ctor_variant<BVH>();
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
  ctx("save") = tag<void, psl::string, Film>(save_film_as_image);
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
  execute(bytecode);
}

}  // namespace pine