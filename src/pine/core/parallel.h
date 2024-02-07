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

  for (auto& thread : threads)
    thread = std::thread([&, tid = tid++]() {
      threadIdx = tid;
      while (true) {
        if (auto index = i++; index < nItems)
          f(index);
        else
          break;
      }
    });

  for (auto& thread : threads)
    thread.join();
}

template <typename F, typename... Args>
void parallel_for(int64_t size, F&& f) {
  parallel_for_impl(size, [&f](int idx) { f(idx); });
}

template <typename F, typename... Args>
void parallel_for(vec2i size, F&& f) {
  parallel_for_impl(area(size), [&f, w = size.x](int idx) { f(vec2i{idx % w, idx / w}); });
}

struct AtomicFloat {
  explicit AtomicFloat(float v = 0) {
    bits = psl::bitcast<uint32_t>(v);
  };
  operator float() const {
    return psl::bitcast<float>(bits);
  }
  AtomicFloat& operator=(float v) {
    bits = psl::bitcast<uint32_t>(v);
    return *this;
  }
  void Add(float v) {
    uint32_t oldBits = bits, newBits;
    do {
      newBits = psl::bitcast<uint32_t>(psl::bitcast<float>(oldBits) + v);
    } while (!bits.compare_exchange_weak(oldBits, newBits));
  }

private:
  std::atomic<uint32_t> bits;
};

}  // namespace pine
