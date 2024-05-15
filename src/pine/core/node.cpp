#include <pine/core/context.h>
#include <pine/core/noise.h>
#include <pine/core/node.h>

namespace pine {

float NodeNoisef::eval(const NodeEvalCtx& ctx) const {
  return fbm(p(ctx), octaves(ctx));
}

vec3 NodeNoise3f::eval(const NodeEvalCtx& ctx) const {
  return fbm3d(p(ctx), octaves(ctx));
}

float NodeCheckerboard::eval(const NodeEvalCtx& ctx) const {
  auto x = fract(p(ctx)) - vec3(ratio);
  return float(x.x * x.y * x.z > 0);
}

vec3 NodeImage::eval(const NodeEvalCtx& ctx) const {
  auto sc = min(vec2i(image->size() * fract(vec2(p(ctx)))), image->size() - vec2i(1));
  return vec3((*image)[sc]);
}
float NodeImagef::eval(const NodeEvalCtx& ctx) const {
  auto sc = min(vec2i(image->size() * fract(vec2(p(ctx)))), image->size() - vec2i(1));
  return (*image)[sc].x;
}

void node_context(Context& ctx) {
  ctx.type<NodeEvalCtx>("NodeCtx")
      .member("p", &NodeEvalCtx::p)
      .member("n", &NodeEvalCtx::n)
      .member("uv", &NodeEvalCtx::uv);
  ctx.type<Nodef>("Nodef");
  ctx.type<Node3f>("Node3f");
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
  ctx.type<NodeUnary<float, 'a'>>("Absf").ctor<Nodef>("abs");
  ctx.type<NodeUnary<vec3, 'a'>>("Abs3f").ctor<Node3f>("abs");
  ctx.type<NodeUnary<float, 's'>>("Sqrf").ctor<Nodef>("sqr");
  ctx.type<NodeUnary<vec3, 's'>>("Sqr3f").ctor<Node3f>("sqr");
  ctx.type<NodeUnary<float, 'r'>>("Sqrtf").ctor<Nodef>("sqrt");
  ctx.type<NodeUnary<vec3, 'r'>>("Sqrt3f").ctor<Node3f>("sqrt");
  ctx.type<NodeUnary<float, 'f'>>("Fractf").ctor<Nodef>("fract");
  ctx.type<NodeUnary<vec3, 'f'>>("Fract3f").ctor<Node3f>("fract");
  ctx.type<NodeComponent>("Comp").ctor<Node3f, int>();
  ctx.type<NodeToVec3>("Vec3").ctor<Nodef>().ctor<Nodef, Nodef, Nodef>();
  ctx.type<NodeNoisef>("Noisef").ctor<Node3f, Nodef>();
  ctx.type<NodeNoise3f>("Noise3f").ctor<Node3f, Nodef>();
  ctx.type<NodeCheckerboard>("Checkerboard").ctor<Node3f>().ctor<Node3f, float>();
  ctx.type<NodeImagef>("Texture").ctor<Node3f, psl::shared_ptr<Image>>();
  ctx.type<NodeImage>("Texture").ctor<Node3f, psl::shared_ptr<Image>>();
  ctx.type<NodeFunction<float>>("Functionf").ctor<psl::function<float(NodeEvalCtx)>>("Function");
  ctx.type<NodeFunction<vec3>>("Function3f").ctor<psl::function<vec3(NodeEvalCtx)>>("Function");
  ctx("+") = +[](Nodef a, Nodef b) { return NodeBinary<float, '+'>{MOVE(a), MOVE(b)}; };
  ctx("-") = +[](Nodef a, Nodef b) { return NodeBinary<float, '-'>{MOVE(a), MOVE(b)}; };
  ctx("*") = +[](Nodef a, Nodef b) { return NodeBinary<float, '*'>{MOVE(a), MOVE(b)}; };
  ctx("/") = +[](Nodef a, Nodef b) { return NodeBinary<float, '/'>{MOVE(a), MOVE(b)}; };
  ctx("^") = +[](Nodef a, Nodef b) { return NodeBinary<float, '^'>{MOVE(a), MOVE(b)}; };
  ctx("-x") = +[](Nodef a) { return NodeUnary<float, '-'>{MOVE(a)}; };
  ctx("+") = +[](Node3f a, Node3f b) { return NodeBinary<vec3, '+'>{MOVE(a), MOVE(b)}; };
  ctx("-") = +[](Node3f a, Node3f b) { return NodeBinary<vec3, '-'>{MOVE(a), MOVE(b)}; };
  ctx("*") = +[](Node3f a, Node3f b) { return NodeBinary<vec3, '*'>{MOVE(a), MOVE(b)}; };
  ctx("/") = +[](Node3f a, Node3f b) { return NodeBinary<vec3, '/'>{MOVE(a), MOVE(b)}; };
  ctx("^") = +[](Node3f a, Node3f b) { return NodeBinary<vec3, '^'>{MOVE(a), MOVE(b)}; };
  ctx("-x") = +[](Node3f a) { return NodeUnary<vec3, '-'>{MOVE(a)}; };
  ctx("*") = +[](Node3f a, Nodef b) { return NodeBinary<vec3, '*'>{MOVE(a), MOVE(b)}; };
  ctx("*") = +[](Nodef a, Node3f b) { return NodeBinary<vec3, '*'>{MOVE(a), MOVE(b)}; };
  ctx("^") = +[](Node3f a, Nodef b) { return NodeBinary<vec3, '^'>{MOVE(a), MOVE(b)}; };
  ctx("/") = +[](Node3f a, Nodef b) { return NodeBinary<vec3, '/'>{MOVE(a), MOVE(b)}; };
  ctx("/") = +[](Nodef a, Node3f b) { return NodeBinary<vec3, '/'>{MOVE(a), MOVE(b)}; };
  ctx("[]") = +[](Node3f a, int index) { return NodeComponent{MOVE(a), index}; };
  ctx("lerp") = +[](Nodef t, Nodef a, Nodef b) {
    return NodeBinary<float, '+'>(
        NodeBinary<float, '*'>(t, MOVE(b)),
        NodeBinary<float, '*'>(NodeBinary<float, '-'>(Nodef(1.0f), t), MOVE(a)));
  };
  ctx("lerp") = +[](Nodef t, Node3f a, Node3f b) {
    return NodeBinary<vec3, '+'>(
        NodeBinary<vec3, '*'>(NodeToVec3(t), MOVE(b)),
        NodeBinary<vec3, '*'>(NodeToVec3(NodeBinary<float, '-'>(Nodef(1.0f), t)), MOVE(a)));
  };
  ctx("lerp") = +[](Node3f t, Node3f a, Node3f b) {
    return NodeBinary<vec3, '+'>(
        NodeBinary<vec3, '*'>(t, MOVE(b)),
        NodeBinary<vec3, '*'>(NodeBinary<vec3, '-'>(Node3f(vec3(1.0f)), t), MOVE(a)));
  };
  ctx.type<Nodef>()
      .ctor_variant<int, float, NodeBinary<float, '+'>, NodeBinary<float, '-'>,
                    NodeBinary<float, '*'>, NodeBinary<float, '/'>, NodeBinary<float, '^'>,
                    NodeUnary<float, '-'>, NodeUnary<float, 'a'>, NodeUnary<float, 's'>,
                    NodeUnary<float, 'r'>, NodeUnary<float, 'f'>, NodeComponent, NodeNoisef,
                    NodeCheckerboard, NodeImagef, psl::function<float(NodeEvalCtx)>>();
  ctx.type<Node3f>()
      .ctor_variant<
          vec3i, vec3, NodeBinary<vec3, '+'>, NodeBinary<vec3, '-'>, NodeBinary<vec3, '*'>,
          NodeBinary<vec3, '/'>, NodeBinary<vec3, '^'>, NodeUnary<vec3, '-'>, NodeUnary<vec3, 'a'>,
          NodeUnary<vec3, 's'>, NodeUnary<vec3, 'r'>, NodeUnary<vec3, 'f'>, NodeToVec3, NodeNoise3f,
          NodePosition, NodeNormal, NodeUV, NodeImage, psl::function<vec3(NodeEvalCtx)>>();
  ctx("pnoise") = overloaded<float, int>(perlin_noise);
  ctx("pnoise") = overloaded<vec2, int>(perlin_noise);
  ctx("pnoise") = overloaded<vec3, int>(perlin_noise);
  ctx("pnoise2d") = overloaded<float, int>(perlin_noise2d);
  ctx("pnoise2d") = overloaded<vec2, int>(perlin_noise2d);
  ctx("pnoise2d") = overloaded<vec3, int>(perlin_noise2d);
  ctx("pnoise3d") = overloaded<float, int>(perlin_noise3d);
  ctx("pnoise3d") = overloaded<vec2, int>(perlin_noise3d);
  ctx("pnoise3d") = overloaded<vec3, int>(perlin_noise3d);
  ctx("fbm") = overloaded<float, int>(fbm);
  ctx("fbm") = overloaded<vec2, int>(fbm);
  ctx("fbm") = overloaded<vec3, int>(fbm);
  ctx("fbm2d") = overloaded<float, int>(fbm2d);
  ctx("fbm2d") = overloaded<vec2, int>(fbm2d);
  ctx("fbm2d") = overloaded<vec3, int>(fbm2d);
  ctx("fbm3d") = overloaded<float, int>(fbm3d);
  ctx("fbm3d") = overloaded<vec2, int>(fbm3d);
  ctx("fbm3d") = overloaded<vec3, int>(fbm3d);
}

}  // namespace pine
