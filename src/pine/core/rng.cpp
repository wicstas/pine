#include <pine/core/context.h>
#include <pine/core/rng.h>

namespace pine {

void rng_context(Context &ctx) {
  ctx.type<RNG>("RNG")
      .ctor<>()
      .method("uniformf", &RNG::uniformf)
      .method("uniform2f", &RNG::uniform2f)
      .method("uniform3f", &RNG::uniform3f);
  static RNG rng;
  ctx("randf") = tag<float>([&]() { return rng.uniformf(); });
}

}  // namespace pine
