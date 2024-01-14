#pragma once

#include <pine/psl/stdint.h>
#include <cstdint>

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
        return -3.40282347e+38f;
    }
    static constexpr float epsilon() {
        return 1.19209290e-7f;
    }
    static constexpr float infinity() {
        // TODO
        return max();
    }
};

template <>
struct numeric_limits<int32_t> {
    static constexpr int32_t max() {
        return 1 << 30;
    }
    static constexpr int32_t lowest() {
        return -(1 << 30);
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


