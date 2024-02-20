#pragma once

#include <pine/core/vecmath.h>
#include <pine/core/log.h>

#include <psl/vector.h>
#include <thread>
#include <atomic>

namespace pine {

inline int n_threads() {
  return std::thread::hardware_concurrency();
}

thread_local inline int threadIdx;

template <typename F, typename... Args>
void parallel_for_impl(int64_t nItems, F&& f) {
  psl::vector<std::thread> threads{static_cast<size_t>(n_threads())};
  std::atomic<int64_t> i{0};
  int tid = 0;
  std::exception_ptr eptr = nullptr;

  for (auto& thread : threads)
    thread = std::thread([&, tid = tid++]() {
      try {
        threadIdx = tid;
        while (true) {
          if (auto index = i++; index < nItems)
            f(index);
          else
            break;

          if (eptr)
            return;
        }
      } catch (...) {
        eptr = std::current_exception();
      }
    });

  for (auto& thread : threads)
    thread.join();
  if (eptr)
    std::rethrow_exception(eptr);
}

template <typename F, typename... Args>
void parallel_for(int64_t size, F&& f) {
  parallel_for_impl(size, [&f](int idx) { f(idx); });
}

template <typename F, typename... Args>
void parallel_for(vec2i size, F&& f) {
  parallel_for_impl(area(size), [&f, w = size.x](int idx) { f(vec2i{idx % w, idx / w}); });
}
template <typename F, typename... Args>
void parallel_for(vec3i64 size, F&& f) {
  auto width = size.x * size.y;
  parallel_for_impl(volume(size), [&](int64_t idx) {
    auto z = idx / width;
    auto xy = idx % width;
    auto y = xy / size.x;
    auto x = xy % size.x;
    f(vec3i64(x, y, z));
  });
}

void parallel_context(Context& ctx);

}  // namespace pine
