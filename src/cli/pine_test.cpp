#include <core/testing.hpp>
#include <core/check.hpp>

using namespace pine;

int main() {
  const auto tests = getTestCollector().getTests();
  runAllTests(tests);
  return 0;
}