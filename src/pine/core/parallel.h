#pragma once

#include <pine/core/vecmath.h>
#include <pine/core/log.h>

#include <psl/vector.h>
#include <thread>
#include <atomic>

#include <xmmintrin.h>
#include <pmmintrin.h>

namespace pine {

inline int n_threads() {
  return std::thread::hardware_concurrency();
}

thread_local inline int threadIdx;

template <typename F, typename... Args>
void ParallelForImpl(int64_t nItems, F&& f) {
  psl::vector<std::thread> threads{static_cast<size_t>(n_threads())};
  std::atomic<int64_t> i{0};
  int tid = 0;

  auto block_size = psl::max(psl::min(int64_t{32}, nItems / n_threads()), int64_t{1});

  for (auto& thread : threads)
    thread = std::thread([&, tid = tid++]() {
      _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
      _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
      threadIdx = tid;
      while (true) {
        auto offset = i += block_size;
        offset -= block_size;
        for (int j = offset; j < offset + block_size; j++) {
          if (j >= nItems)
            return;
          f(j);
        }
      }
    });

  for (auto& thread : threads)
    thread.join();
}

template <typename F, typename... Args>
void ParallelFor(int64_t size, F&& f) {
  ParallelForImpl(size, [&f](int idx) { f(idx); });
}

template <typename F, typename... Args>
void ParallelFor(vec2i size, F&& f) {
  ParallelForImpl(area(size), [&f, w = size.x](int idx) { f(vec2i{idx % w, idx / w}); });
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
