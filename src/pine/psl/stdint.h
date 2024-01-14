#pragma once

#if 1
#include <stdint.h>
#include <stddef.h>
using nullptr_t = decltype(nullptr);

#else
namespace psl {

using int8_t = char;
using int16_t = short;
using int32_t = int;
#ifdef _WIN32
using int64_t = long long;
#else
using int64_t = long;
#endif

using uint8_t = unsigned char;
using uint16_t = unsigned short;
using uint32_t = unsigned int;
#ifdef _WIN32
using uint64_t = unsigned long long;
#else
using uint64_t = unsigned long;
#endif

using size_t = uint64_t;
using ptrdiff_t = int64_t;
using nullptr_t = decltype(nullptr);

}  // namespace psl
#endif

