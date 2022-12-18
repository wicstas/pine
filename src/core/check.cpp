#include <core/check.hpp>

#include <iostream>

namespace pine {

void reportFailedCheck(std::string message,
                       std::source_location sourceLocation) {
  std::cerr << sourceLocation.file_name() << ':' << sourceLocation.line() << ':'
            << sourceLocation.function_name() << '\n'
            << message << '\n';
}

}  // namespace pine
