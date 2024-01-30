#include <psl/chrono.h>

#include <chrono>

namespace psl {

clock::clock() {
  using T = decltype(std::chrono::high_resolution_clock::now());
  time_point = new T;
  *(T*)time_point = std::chrono::high_resolution_clock::now();
}
clock::~clock() {
  using T = decltype(std::chrono::high_resolution_clock::now());
  delete (T*)time_point;
}

float clock::now() {
  using T = decltype(std::chrono::high_resolution_clock::now());
  return std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - *(T*)time_point)
      .count();
}

}  // namespace psl