#pragma once

#include <pine/core/vecmath.h>
#include <pine/core/log.h>

#include <psl/vector.h>

#include <shared_mutex>
#include <thread>
#include <atomic>

namespace pine {

inline int n_threads() {
  return std::thread::hardware_concurrency();
}

thread_local inline int threadIdx;

template <typename F, typename... Args>
void parallel_for_impl(int n_items, F&& f) {
  auto threads = psl::vector<std::thread>(n_threads());
  auto global_index = std::atomic<int>(0);
  auto tid = 0;
  auto mutex = std::shared_mutex();
  auto eptr = std::exception_ptr(nullptr);
  auto batch_count = psl::max<int>(n_threads(), n_items / 64);
  auto batch_size = psl::max(n_items / batch_count, 1);

  for (auto& thread : threads)
    thread = std::thread([&, tid = tid++]() {
      threadIdx = tid;
      try {
        while (true) {
          auto index = (global_index += batch_size) - batch_size;
          auto end_index = psl::min(index + batch_size, n_items);
          for (int i = index; i < end_index; i++)
            f(i);
          if (end_index >= n_items)
            break;
        }
      } catch (...) {
        mutex.lock();
        eptr = std::current_exception();
        mutex.unlock();
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
