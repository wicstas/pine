#pragma once
#include <core/string.hpp>

#include <source_location>

namespace pine {

void reportFailedCheck(
    std::string message,
    std::source_location sourceLocation = std::source_location::current());

#define CHECK_BINARY_OPERATOR(op, a, b)                            \
  do                                                               \
    if (!((a)op(b))) {                                             \
      reportFailedCheck(toString("Check [" #a " " #op " " #b       \
                                 "] failed, where\n  " #a "  =  ", \
                                 a, "\n  " #b "  =  ", b));        \
      abort();                                                     \
    }                                                              \
  while (false)
#define CHECK_EQ(a, b) CHECK_BINARY_OPERATOR(==, a, b)
#define CHECK_NE(a, b) CHECK_BINARY_OPERATOR(!=, a, b)
#define CHECK_LT(a, b) CHECK_BINARY_OPERATOR(<, a, b)
#define CHECK_GT(a, b) CHECK_BINARY_OPERATOR(>, a, b)
#define CHECK_LE(a, b) CHECK_BINARY_OPERATOR(<=, a, b)
#define CHECK_GE(a, b) CHECK_BINARY_OPERATOR(>=, a, b)
#define CHECK(a) CHECK_BINARY_OPERATOR(==, a, true)

#ifdef PINE_DEBUG_BUILD
#define DCHECK_EQ(a, b) CHECK_EQ(a, b)
#define DCHECK_NE(a, b) CHECK_NE(a, b)
#define DCHECK_LT(a, b) CHECK_LT(a, b)
#define DCHECK_GT(a, b) CHECK_GT(a, b)
#define DCHECK_LE(a, b) CHECK_LE(a, b)
#define DCHECK_GE(a, b) CHECK_GE(a, b)
#define DCHECK(a) CHECK(a)
#else
#define DCHECK_EQ(a, b)
#define DCHECK_NE(a, b)
#define DCHECK_LT(a, b)
#define DCHECK_GT(a, b)
#define DCHECK_LE(a, b)
#define DCHECK_GE(a, b)
#define DCHECK(a)
#endif

}  // namespace pine