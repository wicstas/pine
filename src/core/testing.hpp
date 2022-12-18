#pragma once
#include <vector>
#include <string>

namespace pine {

struct Test {
  std::string name;
  void (*function)() = nullptr;
};
using Tests = std::vector<Test>;

class TestCollector {
 public:
  struct NamedTestProxy {
    int operator+(auto testFunction) const {
      namedTest.function = testFunction;
      return 0;
    }

    Test& namedTest;
  };

  NamedTestProxy operator()(std::string testName) {
    tests.push_back(Test(testName, {}));
    return NamedTestProxy(tests.back());
  }

  Tests getTests() const { return tests; }

 private:
  Tests tests;
};

inline TestCollector& getTestCollector() {
  static TestCollector testCollector;
  return testCollector;
};

void runAllTests(const Tests& tests);

#define PINE_CONCAT1(a, b) a##b
#define PINE_CONCAT(a, b) PINE_CONCAT1(a, b)
#define PINE_ANONYMOUS_VARIABLE PINE_CONCAT(anonymousVariable, __LINE__)

#define PINE_TEST(testName) \
  const auto PINE_ANONYMOUS_VARIABLE = getTestCollector()(testName) + []()

}  // namespace pine