#include <psl/string.h>
#include <psl/check.h>

namespace psl {

struct Exception {
  Exception(const auto&... args) : Exception{psl::to_string(args...)} {
  }
  Exception(psl::string message) : message{psl::move(message)} {
  }
  virtual ~Exception() = default;
  virtual const psl::string& what() const {
    return message;
  }

  psl::string message;
};

void throw_check_failure(const char* expr, const char* file, int line, const char* func) {
  throw Exception{"Check `", expr, "` failed[", file, ':', line, ':', func, "()"};
}

}  // namespace psl
