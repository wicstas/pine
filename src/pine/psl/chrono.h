#pragma once

namespace psl {

class clock {
  public:
    clock();
    ~clock();
    clock(const clock&) = delete;
    clock(clock&&) = delete;
    clock& operator=(const clock&) = delete;
    clock& operator=(clock&&) = delete;

    float now();

  private:
    void* time_point;
};

}  // namespace psl

