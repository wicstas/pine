#include <psl/string.h>
#include <psl/check.h>

namespace psl {

void throw_check_failure(const char* expr, const char* file, int line, const char* func) {
  throw MessagedException("Check `", expr, "` failed[", file, ':', line, ':', func, "()");
}

}  // namespace psl
