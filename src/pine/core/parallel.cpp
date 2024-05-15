#include <pine/core/parallel.h>
#include <pine/core/context.h>

namespace pine {

int n_threads() {
  return std::thread::hardware_concurrency();
}

void parallel_context(Context& ctx) {
  ctx("parallel_for") =
      +[](int size, psl::function<void(int)> f) { parallel_for(size, MOVE(f)); };
  ctx("parallel_for2d") =
      +[](vec2i size, psl::function<void(vec2i)> f) { parallel_for(size, MOVE(f)); };
}

}  // namespace pine
