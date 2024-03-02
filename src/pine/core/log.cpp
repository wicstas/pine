#include <pine/core/log.h>

#include <psl/iostream.h>

namespace pine {

void optionally_stop_program_for_stacktrace() {
#ifndef NDEBUG
  psl::abort();
#endif
}

void null_stream(psl::string_view) {
}
void cout_stream(psl::string_view data) {
  psl::cout << data;
}

void (*debug_stream)(psl::string_view data) = cout_stream;
void (*log_stream)(psl::string_view data) = cout_stream;
void (*warning_stream)(psl::string_view data) = cout_stream;
void (*fatal_stream)(psl::string_view data) = cout_stream;

float Timer::elapsed_ms() {
  return (clock.now() - t0) * 1000.0f;
}
float Timer::reset() {
  auto elapsed = elapsed_ms();
  t0 = clock.now();
  return elapsed;
}
void Timer::stop() {
  stop_time = clock.now();
}
void Timer::continue_() {
  t0 += (clock.now() - stop_time);
}

}  // namespace pine