#include <core/testing.hpp>

#include <iostream>

namespace pine {

TestCollector testCollector = {};

void runAllTests(const Tests& tests) {
  std::cerr << "Running " << tests.size() << " tests\n";
  for (const auto& test : tests) {
    std::cerr << "Running test [" << test.name << "]\n";
    test.function();
  }
  std::cerr << "Done\n";
}

}  // namespace pine