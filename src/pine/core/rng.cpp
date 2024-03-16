#include <pine/core/context.h>
#include <pine/core/rng.h>

#include <random>

namespace pine {

void rng_context(Context &ctx) {
  ctx.type<RNG>("RNG")
      .ctor<>()
      .method("nextf", &RNG::nextf)
      .method("next2f", &RNG::next2f)
      .method("next3f", &RNG::next3f);
  static RNG rng;
  static std::uniform_real_distribution<float> dist(0, 1);
  static std::default_random_engine generator;
  ctx("reset_randf") = tag<void, int>([&](int seed) { generator.seed(seed); });
  ctx("randf") = tag<float>([&]() {
    return psl::min(dist(generator), one_minus_epsilon);
    // return rng.nextf();
  });
}

}  // namespace pine
