#pragma once

#include <pine/core/defines.h>

#include <psl/string.h>
#include <psl/system.h>
#include <psl/chrono.h>

namespace pine {

extern void (*debug_stream)(psl::string_view data);
extern void (*log_stream)(psl::string_view data);
extern void (*warning_stream)(psl::string_view data);
extern void (*fatal_stream)(psl::string_view data);

struct Exception : psl::Exception {
  Exception(psl::string message) : message{MOVE(message)} {
  }

  psl::string_view what() const override {
    return message;
  }

private:
  psl::string message;
};

template <typename... Args>
void Debugr(const Args&... args) {
  using psl::to_string;
  debug_stream(to_string(args...));
}
template <typename... Args>
void Debug(const Args&... args) {
  using psl::to_string;
  debug_stream(to_string(args...) + "\n");
}
template <typename... Args>
void Logr(const Args&... args) {
  using psl::to_string;
  log_stream(to_string(args...));
}
template <typename... Args>
void Log(const Args&... args) {
  using psl::to_string;
  log_stream(to_string(args...) + "\n");
}
template <typename... Args>
void Logs(const Args&... args) {
  using psl::to_string;
  log_stream(((to_string(args) + " ") + ...) + "\n");
}
template <typename... Args>
void Warning(const Args&... args) {
  using psl::to_string;
  warning_stream(to_string(args...) + "\n");
}
template <typename... Args>
[[noreturn]] void Fatal(const Args&... args) {
  using psl::to_string;
  fatal_stream(to_string(args...) + "\n");
  psl::abort();
  throw Exception{to_string(args...)};
}

#define CHECK(x)                                                                            \
  if (!(x)) {                                                                               \
    pine::Fatal("[CHECK Failure]Check(", #x, ") failed", '[', __FILE__, ':', __LINE__, ':', \
                __func__, "()]");                                                           \
  }

#define CHECK_IMPL(name, op, a, b)                                                          \
  if (!((a)op(b)))                                                                          \
    pine::Fatal("[" name " Failure]", #a, " = ", a, ", ", #b, " = ", b, '[', __FILE__, ':', \
                __LINE__, ':', __func__, "()]");

#define CHECK_EQ(a, b) CHECK_IMPL("CHECK_EQ", ==, a, b)
#define CHECK_NE(a, b) CHECK_IMPL("CHECK_NE", !=, a, b)
#define CHECK_LT(a, b) CHECK_IMPL("CHECK_LT", <, a, b)
#define CHECK_GT(a, b) CHECK_IMPL("CHECK_GT", >, a, b)
#define CHECK_LE(a, b) CHECK_IMPL("CHECK_LE", <=, a, b)
#define CHECK_GE(a, b) CHECK_IMPL("CHECK_GE", >=, a, b)
#define CHECK_RANGE(t, a, b) \
  CHECK_GE(t, a);            \
  CHECK_LE(t, b)

#ifndef NDEBUG
#define DCHECK(x) CHECK(x)
#define DCHECK_EQ(a, b) CHECK_EQ(a, b)
#define DCHECK_NE(a, b) CHECK_NE(a, b)
#define DCHECK_LT(a, b) CHECK_LT(a, b)
#define DCHECK_GT(a, b) CHECK_GT(a, b)
#define DCHECK_LE(a, b) CHECK_LE(a, b)
#define DCHECK_GE(a, b) CHECK_GE(a, b)
#define DCHECK_RANGE(t, a, b) \
  CHECK_GE(t, a);             \
  CHECK_LE(t, b)
#else
#define DCHECK(x)
#define DCHECK_EQ(a, b)
#define DCHECK_NE(a, b)
#define DCHECK_LT(a, b)
#define DCHECK_GT(a, b)
#define DCHECK_LE(a, b)
#define DCHECK_GE(a, b)
#define DCHECK_RANGE(t, a, b)
#endif

struct Timer {
  float elapsed_ms();
  float reset();
  void stop();
  void continue_();

private:
  psl::clock clock;
  float t0 = clock.now();
  float stop_time = 0.0f;
};

}  // namespace pine