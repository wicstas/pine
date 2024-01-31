#pragma once

namespace psl {

class clock {
public:
  clock();
  ~clock();
  clock(const clock&) = delete;
  clock(clock&& rhs) {
    time_point = rhs.time_point;
    rhs.time_point = nullptr;
  }
  clock& operator=(const clock&) = delete;
  clock& operator=(clock&& rhs) {
    time_point = rhs.time_point;
    rhs.time_point = nullptr;
    return *this;
  }

  float now();

private:
  void* time_point;
};

}  // namespace psl
