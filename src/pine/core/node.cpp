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
  return static_cast<float>(x.x * x.y * x.z > 0);
}

vec3 NodeImage::eval(const NodeEvalCtx& ctx) const {
  return (*image)[clamp(vec2i{image->size() * fract(vec2{p(ctx)})}, vec2i{0},
                        image->size() - vec2i{1})];
}

}  // namespace pine
