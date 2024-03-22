#include <pine/core/context.h>
#include <pine/core/noise.h>
#include <pine/core/node.h>

namespace pine {

float NodeNoisef::eval(const NodeEvalCtx& ctx) const {
  return turbulence(p(ctx), frequency(ctx), octaves(ctx));
}

vec3 NodeNoise3f::eval(const NodeEvalCtx& ctx) const {
  return turbulence3d(p(ctx), frequency(ctx), octaves(ctx));
}

float NodeCheckerboard::eval(const NodeEvalCtx& ctx) const {
  auto x = fract(p(ctx)) - vec3(ratio);
  return float(x.x * x.y * x.z > 0);
}

vec3 NodeImage::eval(const NodeEvalCtx& ctx) const {
  return vec3((*image)[min(vec2i{image->size() * fract(vec2{p(ctx)})}, image->size() - vec2i(1))]);
}
float NodeImagef::eval(const NodeEvalCtx& ctx) const {
  return (*image)[min(vec2i{image->size() * fract(vec2{p(ctx)})}, image->size() - vec2i(1))].x;
}

void node_context(Context& ctx) {
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
  ctx.type<NodeImage>("Texture").ctor<Node3f, psl::shared_ptr<Image>>();
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
  ctx("lerp") = +[](Nodef t, Nodef a, Nodef b) {
    return NodeBinary<float, '+'>(
        NodeBinary<float, '*'>(t, psl::move(b)),
        NodeBinary<float, '*'>(NodeBinary<float, '-'>(Nodef(1.0f), t), psl::move(a)));
  };
  ctx("lerp") = +[](Nodef t, Node3f a, Node3f b) {
    return NodeBinary<vec3, '+'>(
        NodeBinary<vec3, '*'>(NodeToVec3(t), psl::move(b)),
        NodeBinary<vec3, '*'>(NodeToVec3(NodeBinary<float, '-'>(Nodef(1.0f), t)), psl::move(a)));
  };
  ctx("lerp") = +[](Node3f t, Node3f a, Node3f b) {
    return NodeBinary<vec3, '+'>(
        NodeBinary<vec3, '*'>(t, psl::move(b)),
        NodeBinary<vec3, '*'>(NodeBinary<vec3, '-'>(Node3f(vec3(1.0f)), t), psl::move(a)));
  };
  ctx.type<Nodef>()
      .ctor_variant<int, float, NodeConstant<float>, NodeBinary<float, '+'>, NodeBinary<float, '-'>,
                    NodeBinary<float, '*'>, NodeBinary<float, '/'>, NodeBinary<float, '^'>,
                    NodeUnary<float, '-'>, NodeUnary<float, 'a'>, NodeUnary<float, 's'>,
                    NodeUnary<float, 'r'>, NodeUnary<float, 'f'>, NodeComponent, NodeNoisef,
                    NodeCheckerboard>();
  ctx.type<Node3f>()
      .ctor_variant<vec3i, vec3, NodeConstant<vec3>, NodeBinary<vec3, '+'>, NodeBinary<vec3, '-'>,
                    NodeBinary<vec3, '*'>, NodeBinary<vec3, '/'>, NodeBinary<vec3, '^'>,
                    NodeUnary<vec3, '-'>, NodeUnary<vec3, 'a'>, NodeUnary<vec3, 's'>,
                    NodeUnary<vec3, 'r'>, NodeUnary<vec3, 'f'>, NodeToVec3, NodeNoise3f,
                    NodePosition, NodeNormal, NodeUV, NodeImage>();
  ctx("perlin_noise") = +[](vec3 np, float frequency) { return perlin_noise(np, frequency); };
  ctx("perlin_noise3d") = +[](vec3 np, float frequency) { return perlin_noise3d(np, frequency); };
  ctx("turbulence") = turbulence;
  ctx("turbulence3d") = turbulence3d;
}

}  // namespace pine
