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
  ctx("pi") = pi;
  ctx("E") = psl::E;
  ctx("min") = psl::min<float>;
  ctx("max") = psl::max<float>;
  ctx("abs") = psl::abs<float>;
  ctx("clamp") = psl::clamp<float>;
  ctx("lerp") = psl::lerp<float, float>;
  ctx("sqr") = psl::sqr<float>;
  ctx("sqrt") = psl::sqrt<float>;
  ctx("floor") = psl::floor<float>;
  ctx("ceil") = psl::ceil<float>;
  ctx("pow") = psl::pow<float>;
  ctx("exp") = psl::exp<float>;
  ctx("log") = psl::log<float>;
  ctx("sin") = psl::sin<float>;
  ctx("cos") = psl::cos<float>;
  ctx("tan") = psl::tan<float>;
  ctx("acos") = psl::acos<float>;
  ctx("asin") = psl::asin<float>;
  ctx("atan2") = psl::atan2<float>;
  ctx.type<vec2i, Context::Complex>("vec2i")
      .ctor<int>()
      .ctor<int, int>()
      .member("x", &vec2i::x)
      .member("y", &vec2i::y);
  ctx.type<vec3i, Context::Complex>("vec3i")
      .ctor<int>()
      .ctor<int, int, int>()
      .member("x", &vec3i::x)
      .member("y", &vec3i::y)
      .member("z", &vec3i::z);
  ctx.type<vec2, Context::Complex>("vec2")
      .ctor<float, float>()
      .ctor_variant<vec2i, int, float>()
      .member("x", &vec2::x)
      .member("y", &vec2::y);
  ctx.type<vec3, Context::Complex>("vec3")
      .ctor<float, float, float>()
      .ctor_variant<vec3i, int, float>()
      .member("x", &vec3::x)
      .member("y", &vec3::y)
      .member("z", &vec3::z);
  ctx.type<vec4, Context::Complex>("vec4")
      .ctor<float, float, float, float>()
      .ctor_variant<vec4i, int, float>()
      .member("x", &vec4::x)
      .member("y", &vec4::y)
      .member("z", &vec4::z)
      .member("w", &vec4::w);
  ctx.type<mat3>("mat3").ctor<vec3, vec3, vec3>().ctor<mat4>();
  ctx.type<mat4>("mat4").ctor<vec4, vec4, vec4, vec4>();
  ctx("*") = +[](const mat3& a, vec3 b) { return a * b; };
  ctx("*") = +[](const mat3& a, const mat3& b) { return a * b; };
  ctx("*") = +[](const mat4& a, vec4 b) { return a * b; };
  ctx("*") = +[](const mat4& a, const mat4& b) { return a * b; };
  ctx("translate") = overloaded<vec3>(translate);
  ctx("scale") = overloaded<vec3>(scale);
  ctx("look_at") = look_at;
  ctx("*") = overloads_set<Overloads<vec2i, vec3i>, Overloads<int>>(psl::mul_);
  ctx("/") = overloads_set<Overloads<vec2i, vec3i>, Overloads<int>>(psl::div_);
  ctx("*") = overloads_set<Overloads<int>, Overloads<vec2i, vec3i>>(psl::mul_);
  ctx("/") = overloads_set<Overloads<int>, Overloads<vec2i, vec3i>>(psl::div_);
  ctx("*") = overloads_set<Overloads<vec2, vec3, vec4>, Overloads<float>>(psl::mul_);
  ctx("/") = overloads_set<Overloads<vec2, vec3, vec4>, Overloads<float>>(psl::div_);
  ctx("*") = overloads_set<Overloads<float>, Overloads<vec2, vec3, vec4>>(psl::mul_);
  ctx("/") = overloads_set<Overloads<float>, Overloads<vec2, vec3, vec4>>(psl::div_);
  ctx("*=") = overloads_set<Overloads<vec2i, vec3i>, Overloads<int>>(psl::mule_);
  ctx("/=") = overloads_set<Overloads<vec2i, vec3i>, Overloads<int>>(psl::dive_);
  ctx("*=") = overloads_set<Overloads<vec2, vec3, vec4>, Overloads<float>>(psl::mule_);
  ctx("/=") = overloads_set<Overloads<vec2, vec3, vec4>, Overloads<float>>(psl::dive_);
  ctx("normalize") = normalize<vec2>;
  ctx("normalize") = normalize<vec3>;
  ctx("length") = length<vec2>;
  ctx("length") = length<vec3>;
  ctx("distance") = distance<vec3>;
  ctx("distance") = distance<vec3>;
  ctx("dot") = overloaded<vec2, vec2>(dot<float>);
  ctx("dot") = overloaded<vec3, vec3>(dot<float>);
  ctx("cross") = overloaded<vec3, vec3>(dot<float>);
  ctx("fract") = overloaded<vec2>(fract<float>);
  ctx("fract") = overloaded<vec3>(fract<float>);
  ctx("floor") = overloaded<vec2>(floor<float>);
  ctx("floor") = overloaded<vec3>(floor<float>);
  ctx("ceil") = overloaded<vec2>(ceil<float>);
  ctx("ceil") = overloaded<vec3>(ceil<float>);
  ctx("sqrt") = overloaded<vec2>(sqrt<float>);
  ctx("sqrt") = overloaded<vec3>(sqrt<float>);
  ctx("exp") = overloaded<vec2>(exp<float>);
  ctx("exp") = overloaded<vec3>(exp<float>);
  ctx("abs") = overloaded<vec2>(abs<float>);
  ctx("abs") = overloaded<vec3>(abs<float>);
  ctx("min") = overloaded<vec2, vec2>(min<float>);
  ctx("min") = overloaded<vec3, vec3>(min<float>);
  ctx("max") = overloaded<vec2, vec2>(max<float>);
  ctx("max") = overloaded<vec3, vec3>(max<float>);
  ctx("clamp") = overloaded<vec2, vec2, vec2>(clamp<float>);
  ctx("clamp") = overloaded<vec3, vec3, vec3>(clamp<float>);
  ctx("lerp") = overloaded<vec2, vec2, vec2>(lerp<float>);
  ctx("lerp") = overloaded<vec3, vec3, vec3>(lerp<float>);
  ctx("coordinate_system") = overloaded<vec3>(coordinate_system);
  ctx("rotate_x") = rotate_x;
  ctx("rotate_y") = rotate_y;
  ctx("rotate_z") = rotate_z;
  ctx("rotate") = rotate;
  ctx("identity3x3") = mat3::identity;
  ctx("identity4x4") = mat4::identity;
  ctx.type<RNG>("RNG")
      .ctor<>()
      .method("uniformf", &RNG::uniformf)
      .method("uniform2f", &RNG::uniform2f)
      .method("uniform3f", &RNG::uniform3f);
  ctx.type<Sphere>("Sphere").ctor<vec3, float>();
  ctx.type<Plane>("Plane").ctor<vec3, vec3>();
  ctx.type<Disk>("Disk").ctor<vec3, vec3, float>();
  ctx.type<Line>("Line").ctor<vec3, vec3, float>();
  ctx.type<Rect>("Rect").ctor<vec3, vec3, vec3>();
  ctx.type<Triangle>("Triangle").ctor<vec3, vec3, vec3>();
  ctx.type<TriangleMesh>("TriangleMesh").method("apply", &TriangleMesh::apply);
  ctx("load_mesh") = +[](psl::string filename) { return load_mesh(filename); };
  ctx.type<Shape>("Shape").ctor_variant<Sphere, Plane, Disk, Line, Triangle, Rect, TriangleMesh>();
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
  ctx.type<Camera>("Camera").ctor_variant<ThinLenCamera>().method(
      "film", +[](Camera& camera) { return psl::ref{camera.film()}; });
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