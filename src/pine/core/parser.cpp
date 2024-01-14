#include <pine/core/grammar.h>
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
  ctx.type<bool, Class::BoolLike>("bool").ctor<>();
  ctx.type<int, Class::IntLike>("int").ctors<bool, float>();
  ctx.type<float, Class::FloatLike>("float").ctors<bool, int>();
  ctx("pi") = pi;
  ctx("E") = psl::E;
  ctx("%") = +[](int a, int b) { return a % b; };
  ctx("+") = +[](int a, float b) { return a + b; };
  ctx("+") = +[](float a, int b) { return a + b; };
  ctx("-") = +[](int a, float b) { return a - b; };
  ctx("-") = +[](float a, int b) { return a - b; };
  ctx("*") = +[](int a, float b) { return a * b; };
  ctx("*") = +[](float a, int b) { return a * b; };
  ctx("/") = +[](int a, float b) { return a / b; };
  ctx("/") = +[](float a, int b) { return a / b; };
  ctx("+=") = +[](float& a, int b) -> float& { return a += b; };
  ctx("-=") = +[](float& a, int b) -> float& { return a -= b; };
  ctx("*=") = +[](float& a, int b) -> float& { return a *= b; };
  ctx("/=") = +[](float& a, int b) -> float& { return a /= b; };
  ctx("=") = +[](float& a, int b) -> float& { return a = b; };
  ctx.type<psl::string>("string");
  ctx("+=") = +[](psl::string& a, psl::string b) -> psl::string& { return a += b; };
  ctx("+") = +[](psl::string a, psl::string b) { return a + b; };
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
  ctx.type<vec2i, Class::ComplexLike>("vec2i")
      .ctor<int>()
      .ctor<int, int>()
      .var("x", &vec2i::x)
      .var("y", &vec2i::y);
  ctx.type<vec3i, Class::ComplexLike>("vec3i")
      .ctor<int>()
      .ctor<int, int, int>()
      .var("x", &vec3i::x)
      .var("y", &vec3i::y)
      .var("z", &vec3i::z);
  ctx.type<vec2, Class::ComplexLike>("vec2")
      .ctor<float, float>()
      .ctors<vec2i, int, float>()
      .var("x", &vec2::x)
      .var("y", &vec2::y);
  ctx.type<vec3, Class::ComplexLike>("vec3")
      .ctor<float, float, float>()
      .ctors<vec3i, int, float>()
      .var("x", &vec3::x)
      .var("y", &vec3::y)
      .var("z", &vec3::z);
  ctx.type<vec4, Class::ComplexLike>("vec4")
      .ctor<float, float, float, float>()
      .ctors<vec4i, int, float>()
      .var("x", &vec4::x)
      .var("y", &vec4::y)
      .var("z", &vec4::z)
      .var("w", &vec4::w);
  ctx.type<mat3>("mat3").ctor<vec3, vec3, vec3>().ctor<mat4>();
  ctx.type<mat4>("mat4").ctor<vec4, vec4, vec4, vec4>();
  ctx("*") = +[](const mat3& a, vec3 b) { return a * b; };
  ctx("*") = +[](const mat3& a, const mat3& b) { return a * b; };
  ctx("*") = +[](const mat4& a, vec4 b) { return a * b; };
  ctx("*") = +[](const mat4& a, const mat4& b) { return a * b; };
  ctx("translate") = overload<mat4, vec3>(translate);
  ctx("scale") = overload<mat4, vec3>(scale);
  ctx("look_at") = look_at;
  add_f<Overloads<vec2i, vec3i, vec2, vec3, vec4>, Overloads<int, float>>(ctx, "*", psl::mul_);
  add_f<Overloads<vec2i, vec3i, vec2, vec3, vec4>, Overloads<int, float>>(ctx, "/", psl::div_);
  add_f<Overloads<int, float>, Overloads<vec2i, vec3i, vec2, vec3, vec4>>(ctx, "*", psl::mul_);
  add_f<Overloads<int, float>, Overloads<vec2i, vec3i, vec2, vec3, vec4>>(ctx, "/", psl::div_);
  add_f<Overloads<vec2i, vec3i>, Overloads<int>>(ctx, "*=", psl::mule_);
  add_f<Overloads<vec2i, vec3i>, Overloads<int>>(ctx, "/=", psl::dive_);
  add_f<Overloads<vec2, vec3, vec4>, Overloads<int, float>>(ctx, "*=", psl::mule_);
  add_f<Overloads<vec2, vec3, vec4>, Overloads<int, float>>(ctx, "/=", psl::dive_);
  ctx("normalize") = normalize<vec2>;
  ctx("normalize") = normalize<vec3>;
  ctx("length") = length<vec2>;
  ctx("length") = length<vec3>;
  ctx("distance") = distance<vec3>;
  ctx("distance") = distance<vec3>;
  ctx("dot") = overload<float, vec2, vec2>(dot<float>);
  ctx("dot") = overload<float, vec3, vec3>(dot<float>);
  ctx("cross") = overload<float, vec3, vec3>(dot<float>);
  ctx("fract") = overload<vec2, vec2>(fract<float>);
  ctx("fract") = overload<vec3, vec3>(fract<float>);
  ctx("floor") = overload<vec2, vec2>(floor<float>);
  ctx("floor") = overload<vec3, vec3>(floor<float>);
  ctx("ceil") = overload<vec2, vec2>(ceil<float>);
  ctx("ceil") = overload<vec3, vec3>(ceil<float>);
  ctx("sqrt") = overload<vec2, vec2>(sqrt<float>);
  ctx("sqrt") = overload<vec3, vec3>(sqrt<float>);
  ctx("exp") = overload<vec2, vec2>(exp<float>);
  ctx("exp") = overload<vec3, vec3>(exp<float>);
  ctx("abs") = overload<vec2, vec2>(abs<float>);
  ctx("abs") = overload<vec3, vec3>(abs<float>);
  ctx("min") = overload<vec2, vec2, vec2>(min<float>);
  ctx("min") = overload<vec3, vec3, vec3>(min<float>);
  ctx("max") = overload<vec2, vec2, vec2>(max<float>);
  ctx("max") = overload<vec3, vec3, vec3>(max<float>);
  ctx("clamp") = overload<vec2, vec2, vec2, vec2>(clamp<float>);
  ctx("clamp") = overload<vec3, vec3, vec3, vec3>(clamp<float>);
  ctx("lerp") = overload<vec2, vec2, vec2, vec2>(lerp<float>);
  ctx("lerp") = overload<vec3, vec3, vec3, vec3>(lerp<float>);
  ctx("coordinate_system") = overload<mat3, vec3>(coordinate_system);
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
  ctx.type<Shape>("Shape").ctors<Sphere, Plane, Disk, Line, Triangle, Rect, TriangleMesh>();
  ctx.type<PointLight>("PointLight").ctor<vec3, vec3>();
  ctx.type<DirectionalLight>("DirectionalLight").ctor<vec3, vec3>();
  ctx.type<Light>("Light").ctors<PointLight, DirectionalLight>();
  ctx.type<Sky>("Sky").ctor<vec3>();
  ctx.type<Atmosphere>("Atmosphere").ctor<vec3, vec3>();
  ctx.type<ImageSky>("ImageSky")
      .ctor<psl::shared_ptr<Image>>()
      .ctor<psl::shared_ptr<Image>, vec3>();
  ctx("ImageSky") = tag<ImageSky, psl::string>([&](psl::string filename) {
    return ImageSky{ctx.call("load_image", filename).as<psl::shared_ptr<Image>>()};
  });
  ctx("ImageSky") = tag<ImageSky, psl::string, vec3>([&](psl::string filename, vec3 tint) {
    return ImageSky{ctx.call("load_image", filename).as<psl::shared_ptr<Image>>(), tint};
  });
  ctx.type<EnvironmentLight>("EnvironmentLight").ctors<Atmosphere, Sky, ImageSky>();
  ctx.type<NodePosition>("Position").ctor<>();
  ctx.type<NodeNormal>("Normal").ctor<>();
  ctx.type<NodeUV>("UV").ctor<>();
  ctx.type<NodeConstant<float>>("Constantf").ctors<float>();
  ctx.type<NodeConstant<vec3>>("Constant3f").ctors<vec3>();
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
  ctx("Texture") = tag<NodeImage, Node3f, psl::string>([&](Node3f p, psl::string filename) {
    return NodeImage{p, ctx.call("load_image", filename).as<psl::shared_ptr<Image>>()};
  });
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
      .ctors<int, float, NodeConstant<float>, NodeBinary<float, '+'>, NodeBinary<float, '-'>,
             NodeBinary<float, '*'>, NodeBinary<float, '/'>, NodeBinary<float, '^'>,
             NodeUnary<float, '-'>, NodeUnary<float, 'a'>, NodeUnary<float, 's'>,
             NodeUnary<float, 'r'>, NodeUnary<float, 'f'>, NodeComponent, NodeNoisef,
             NodeCheckerboard>();
  ctx.type<Node3f>("Node3f")
      .ctors<vec3i, vec3, NodeConstant<vec3>, NodeBinary<vec3, '+'>, NodeBinary<vec3, '-'>,
             NodeBinary<vec3, '*'>, NodeBinary<vec3, '/'>, NodeBinary<vec3, '^'>,
             NodeUnary<vec3, '-'>, NodeUnary<vec3, 'a'>, NodeUnary<vec3, 's'>, NodeUnary<vec3, 'r'>,
             NodeUnary<vec3, 'f'>, NodeToVec3, NodeNoise3f, NodePosition, NodeNormal, NodeUV,
             NodeImage>();
  ctx.type<DiffuseBSDF>("DiffuseBSDF").ctor<Node3f>();
  ctx.type<ConductorBSDF>("ConductorBSDF").ctor<Node3f, Nodef>();
  ctx.type<DielectricBSDF>("DielectricBSDF").ctor<Node3f, Nodef, Nodef>();
  ctx.type<SpecularReflectionBSDF>("SpecularReflectionBSDF").ctor<Node3f>();
  ctx.type<SpecularRefrectionBSDF>("SpecularRefrectionBSDF").ctor<Node3f, Nodef>();
  ctx.type<BSDF>("BSDF").ctors<DiffuseBSDF, ConductorBSDF, DielectricBSDF>();
  ctx.type<LayeredMaterial>("Layered").ctor<BSDF>().ctor<BSDF, BSDF>();
  ctx.type<DiffuseMaterial>("Diffuse").ctor<Node3f>();
  ctx.type<MetalMaterial>("Metal").ctor<Node3f, Nodef>();
  ctx.type<GlassMaterial>("Glass").ctor<Node3f, Nodef>();
  ctx.type<GlossyMaterial>("Glossy").ctor<Node3f, Node3f, Nodef>();
  ctx.type<MirrorMaterial>("Mirror").ctor<Node3f>();
  ctx.type<WaterMaterial>("Water").ctor<Node3f, Nodef>();
  ctx.type<EmissiveMaterial>("Emissive").ctor<Node3f>();
  ctx.type<Material>("Material")
      .ctors<LayeredMaterial, DiffuseMaterial, MetalMaterial, GlassMaterial, GlossyMaterial,
             MirrorMaterial, WaterMaterial, EmissiveMaterial>();
  ctx.type<Film>("Film")
      .ctor<vec2i>()
      .method("scale", &Film::scale)
      .method("offset", &Film::offset);
  ctx.type<ThinLenCamera>("ThinLenCamera")
      .ctor<Film, vec3, vec3, float>()
      .ctor<Film, vec3, vec3, float, float, float>();
  ctx.type<Camera>("Camera").ctors<ThinLenCamera>().method(
      "film", +[](Camera& camera) { return psl::ref{camera.film()}; });
  ctx.type<Scene>("Scene")
      .ctor<>()
      .var("camera", &Scene::camera)
      .method("add", &Scene::add_material)
      .method("add", overload<void, Shape, psl::string>(&Scene::add_geometry))
      .method("add", overload<void, Shape, Material>(&Scene::add_geometry))
      .method("add", overload<void, Light>(&Scene::add_light))
      .method("set", &Scene::set_camera)
      .method("set", &Scene::set_env_light)
      .method("reset", &Scene::reset);
  ctx("add_box") = overload<void, Scene&, mat4, Material>(add_box);
  ctx("add_box") = overload<void, Scene&, mat4, psl::string>(add_box);
  ctx.type<UniformSampler>("UniformSampler").ctor<int>();
  ctx.type<HaltonSampler>("HaltonSampler").ctor<int, vec2i>();
  ctx.type<Sampler>("Sampler").ctors<UniformSampler, HaltonSampler>();
  ctx.type<BVH>("BVH").ctor();
  ctx.type<Accel>("Accel").ctors<BVH>();
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
  ctx.type<LightSampler>("LightSampler").ctors<UniformLightSampler>();
  ctx("save") = tag<void, psl::string, Film>(save_film_as_image);
  add_f<Overloads<bool, int, float, vec2i, vec3i, vec2, vec3, psl::string>>(ctx, "to_string",
                                                                            [](auto&& x) {
                                                                              using psl::to_string;
                                                                              return to_string(x);
                                                                            });
  ctx("print") =
      tag<Variable, const psl::vector<Variable>&>([&](const psl::vector<Variable>& vars) {
        auto str = psl::string{};
        for (auto&& var : vars)
          str += ctx.call("to_string", var).as<psl::string>();
        Logr(str);
        return Variable{};
      });
  ctx("println") =
      tag<Variable, const psl::vector<Variable>&>([&](const psl::vector<Variable>& vars) {
        auto str = psl::string{};
        for (auto&& var : vars)
          str += ctx.call("to_string", var).as<psl::string>();
        Log(str);
        return Variable{};
      });
  ctx("assert") = +[](bool x) {
    if (!x)
      exception("Assertion failed");
  };

  return ctx;
}

void interpret(Context& ctx, psl::string source) {
  auto [file, sl] = parse_as_block(source);
  ctx.sl = psl::move(sl);
  file.eval(ctx);
}

}  // namespace pine