#pragma once
#include <chrono>

namespace pine {

struct Timer {
  using clock = std::chrono::high_resolution_clock;

  float elapsed() const {
    const auto t1 = clock::now();
    auto d = t1 - t0;
    return std::chrono::duration<float>(t1 - t0).count();
  }

 private:
  clock::time_point t0 = clock::now();
};

}  // namespace pine
