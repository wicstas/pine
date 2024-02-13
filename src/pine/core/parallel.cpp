#include <pine/core/parallel.h>
#include <pine/core/context.h>

namespace pine {

void parallel_context(Context& ctx) {
  ctx("parallel_for") =
      +[](int size, psl::function<void(int)> f) { parallel_for(size, psl::move(f)); };
  ctx("parallel_for2d") =
      +[](vec2i size, psl::function<void(vec2i)> f) { parallel_for(size, psl::move(f)); };
}

}  // namespace pine
