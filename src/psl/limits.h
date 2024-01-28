#pragma once

#include <psl/stdint.h>

namespace psl {

template <typename T>
struct numeric_limits;

template <>
struct numeric_limits<float> {
  static constexpr float max() {
    return 3.40282347e+38f;
  }
  static constexpr float min() {
    return 1.17549435e-38f;
  }
  static constexpr float lowest() {
    return -max();
  }
  static constexpr float epsilon() {
    return 1.19209290e-7f;
  }
  static constexpr float infinity() {
    return max();
  }
};

template <>
struct numeric_limits<double> {
  static constexpr double max() {
    return 1.7976931348623157e+308;
  }
  static constexpr double min() {
    return 2.2250738585072014e-308;
  }
  static constexpr double lowest() {
    return -max();
  }
  static constexpr double epsilon() {
    return 2.2204460492503131e-16;
  }
  static constexpr double infinity() {
    return max();
  }
};

template <>
struct numeric_limits<int32_t> {
  static constexpr int32_t max() {
    return 0x7fffffff;
  }
  static constexpr int32_t lowest() {
    return -max();
  }
};

template <>
struct numeric_limits<int64_t> {
  static constexpr int64_t max() {
    return 0x7fffffffffffffff;
  }
  static constexpr int64_t lowest() {
    return -max();
  }
};

template <>
struct numeric_limits<uint64_t> {
  static constexpr uint64_t max() {
    return uint64_t(-1);
  }
  static constexpr uint64_t lowest() {
    return 0;
  }
};

template <>
struct numeric_limits<uint32_t> {
  static constexpr uint32_t max() {
    return uint32_t(-1);
  }
  static constexpr uint32_t lowest() {
    return 0;
  }
};

template <>
struct numeric_limits<uint16_t> {
  static constexpr uint16_t max() {
    return uint16_t(-1);
  }
  static constexpr uint16_t lowest() {
    return 0;
  }
};

template <>
struct numeric_limits<uint8_t> {
  static constexpr uint8_t max() {
    return uint8_t(-1);
  }
  static constexpr uint8_t lowest() {
    return 0;
  }
};

}  // namespace psl
