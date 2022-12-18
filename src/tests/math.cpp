#include <core/math.hpp>
#include <core/testing.hpp>
#include <core/check.hpp>

namespace pine {

PINE_TEST("lengthSquared()") {
  CHECK_EQ(lengthSquared(1), 1);
  CHECK_EQ(lengthSquared(2), 4);
  CHECK_EQ(lengthSquared(-2), 4);
};

PINE_TEST("sum()") {
  CHECK_EQ(sum(1), 1);
  CHECK_EQ(sum(1, 2), 3);
  CHECK_EQ(sum(1, -2, 3), 2);
};

PINE_TEST("dimensionOf") {
  static_assert(dimensionOf<int, int> == 1, "");
  static_assert(dimensionOf<int, Vector<int, 1>> == 1, "");
  static_assert(dimensionOf<int, Vector<int, 2>> == 2, "");
  static_assert(dimensionOf<Vector<int, 2>, Vector<int, 2>> == 1, "");
  static_assert(dimensionOf<int, Vector2<Vector3<int>>> == 2, "");
};

PINE_TEST("Vector constructors") {
  auto x = vec3i();
  CHECK_EQ(x[0], 0);
  CHECK_EQ(x[1], 0);
  CHECK_EQ(x[2], 0);
  x = vec3i(2);
  CHECK_EQ(x[0], 2);
  CHECK_EQ(x[1], 2);
  CHECK_EQ(x[2], 2);
  x = vec3i(1, 2, 3);
  CHECK_EQ(x[0], 1);
  CHECK_EQ(x[1], 2);
  CHECK_EQ(x[2], 3);
  x = vec3i(vec2i(1, 2), 3);
  CHECK_EQ(x[0], 1);
  CHECK_EQ(x[1], 2);
  CHECK_EQ(x[2], 3);
  x = vec3i(vec4i(1, 2, 3, 4));
  CHECK_EQ(x[0], 1);
  CHECK_EQ(x[1], 2);
  CHECK_EQ(x[2], 3);
  x = vec3i(vec2(1.1, 2.9), 3);
  CHECK_EQ(x[0], 1);
  CHECK_EQ(x[1], 2);
  CHECK_EQ(x[2], 3);
  x = vec3i(vec3(1.1, 2.9, 3));
  CHECK_EQ(x[0], 1);
  CHECK_EQ(x[1], 2);
  CHECK_EQ(x[2], 3);
};

PINE_TEST("Vector operations") {
  auto x = vec3i(1, 2, 3);
  auto y = vec3(10, 20, 30);
  CHECK_EQF(x + y, vec3(11, 22, 33));
  CHECK_EQF(x - y, vec3(-9, -18, -27));
  CHECK_EQF(x * y, vec3(10, 40, 90));
  CHECK_EQF(x / y, vec3(0.1, 0.1, 0.1));
  CHECK_EQF(x * 2.0f, vec3(2, 4, 6));
  CHECK_EQF(x / 2.0f, vec3(0.5, 1.0, 1.5));
  CHECK_EQF(2.0f * x, vec3(2, 4, 6));
  CHECK_EQF(2.0f / x, vec3(2, 1, 2.0f / 3));
  CHECK_EQF(innerProduct(vec2(1, 2), vec2(10, 20)), 50);
  CHECK_EQF(dot(vec2(1, 2), vec2(10, 20)), 50);
  CHECK_EQF(lengthSquared(vec2(1, 2)), 5);
  CHECK_EQF(length(vec2(3, 4)), 5);
};

PINE_TEST("Matrix operations") {
  auto x = vec2(1, 2);
  auto m = mat2(vec2(1, 2), vec2(3, 4));
  auto id = mat2::identity();
  auto s = scale(vec2(10, 20));
  auto t = translate(vec2(10, 20));
  CHECK_EQF(m, m * id);
  CHECK_EQF(m, id * m);
  CHECK_EQF(m * x, vec2(7, 10));
  CHECK_EQF(id * x, x);
  CHECK_EQF(s * x, vec2(10, 40));
  CHECK_EQF(t * vec3(x, 1), vec3(11, 22, 1));
  CHECK_EQF(lengthSquared(m), 30);
};

}  // namespace pine