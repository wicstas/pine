#include <psl/system.h>
#include <psl/string.h>

#include <cxxabi.h>
#include <stdlib.h>

namespace psl {

void abort() {
  ::abort();
}

string demangle(const char* mangled_type_name) {
  auto status = 0;
  auto result = psl::unique_ptr<char[], void (*)(void*)>(
      abi::__cxa_demangle(mangled_type_name, 0, 0, &status), psl::free);
  return result ? psl::string(result.get()) : "null";
}

string stacktrace() {
  return "Stacktrace unavailable";
}

}  // namespace psl
