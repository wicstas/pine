#include <pine/core/log.h>

#include <psl/iostream.h>

namespace pine {

void stop_program() {
  // #ifndef NDEBUG
  psl::abort();
  // #endif
  // throw FatalException{};
}

void cout_stream(psl::string_view data) {
  psl::cout << data;
}

void (*debug_stream)(psl::string_view data) = cout_stream;
void (*log_stream)(psl::string_view data) = cout_stream;
void (*warning_stream)(psl::string_view data) = cout_stream;
void (*fatal_stream)(psl::string_view data) = cout_stream;

double Timer::ElapsedMs() {
  return (clock.now() - t0) * 1000.0;
}
double Timer::Reset() {
  double elapsed = ElapsedMs();
  t0 = clock.now();
  return elapsed;
}

}  // namespace pine