#include <core/math.hpp>
#include <core/testing.hpp>
#include <core/check.hpp>

namespace pine {

PINE_TEST("common functions") {
  CHECK_EQ(sqr(1), 1);
  CHECK_EQ(sqr(2), 4);
  CHECK_EQ(sqr(-2), 4);
  CHECK_EQ(lengthSquared(1), 1);
  CHECK_EQ(lengthSquared(2), 4);
  CHECK_EQ(lengthSquared(-2), 4);
  CHECK_EQF(length(1), 1);
  CHECK_EQF(length(2), 2);
  CHECK_EQF(length(-2), 2);
  CHECK_EQ(sum(1), 1);
  CHECK_EQ(sum(1, 2), 3);
  CHECK_EQ(sum(1, -2, 3), 2);
  CHECK_EQ(max(1, -2), 1);
  CHECK_EQ(max(1, -2, 3), 3);
  CHECK_EQ(min(1, -2), -2);
  CHECK_EQ(min(1, -2, 3), -2);
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
  CHECK_EQF(innerProduct(vec2(1, 2), vec2(10, 20)), 50);
  CHECK_EQF(dot(vec2(1, 2), vec2(10, 20)), 50);
  CHECK_EQF(lengthSquared(vec2(1, 2)), 5);
  CHECK_EQF(length(vec2(3, 4)), 5);
  CHECK_EQF(cross(vec3(1, 2, 3), vec3(2, 3, 4)), vec3(-1, 2, -1));
  CHECK_EQF(shuffle(vec3(10, 20, 30), vec3i(2, 1, 0)), vec3(30, 20, 10));
  CHECK_EQF((oneAt<int, 3, 0>()), vec3i(1, 0, 0));
  CHECK_EQF((oneAt<int, 3, 1>()), vec3i(0, 1, 0));
  CHECK_EQF(abs(vec2(3, -2)), vec2(3, 2));
  CHECK_EQF(sqrt(vec2(4, 9)), vec2(2, 3));
  CHECK_EQF(pow(vec2(2, 3), 2.0f), vec2(4, 9));
  CHECK_EQF(exp(vec2(1, 2)), vec2(E, E * E));
  CHECK_EQF(log(vec2(E, E * E)), vec2(1, 2));
  CHECK_EQF(clamp(7, 1, 6), 6);
  CHECK_EQF(clamp(-3, 1, 6), 1);
  CHECK_EQF(clamp(vec2(10, 20), vec2(5, 5), vec2(15, 15)), vec2(10, 15));
  CHECK_EQF(clamp(vec2(0, 10), vec2(5, 5), vec2(15, 15)), vec2(5, 10));
  CHECK_EQF(lerp(1.0, 2.0, 0.4), 1.4);
  CHECK_EQF(lerp(vec2(1.0, 10.0), vec2(2.0, 20.0), 0.4f), vec2(1.4, 14));
};

PINE_TEST("Matrix operations") {
  auto x = vec2(1, 2);
  auto x3 = vec3(1, 2, 3);
  auto x4 = vec4(1, 2, 3, 4);
  auto m = mat2(vec2(1, 2), vec2(3, 4));
  auto m3 = mat3(vec3(1, 2, 3), vec3(2, 5, 4), vec3(3, 7, 1));
  auto m4 = mat4(vec4(1, 2, 3, 9), vec4(2, 5, 4, 1), vec4(3, 7, 1, 8),
                 vec4(17, 4, 6, 7));
  auto id = mat2::identity();
  auto s = scale(vec2(10, 20));
  auto t = translate(vec2(10, 20));
  CHECK_EQF(mat2::identity(), mat2(vec2(1, 0), vec2(0, 1)));
  CHECK_EQF((Matrix<int, 2, 3>::identity()),
            (Matrix<int, 2, 3>(vec2i(1, 0), vec2i(0, 1), vec2i(0, 0))));
  CHECK_EQF((Matrix<int, 3, 2>::identity()),
            (Matrix<int, 3, 2>(vec3i(1, 0, 0), vec3i(0, 1, 0))));
  CHECK_EQF(m, m * id);
  CHECK_EQF(m, id * m);
  CHECK_EQF(m * x, vec2(7, 10));
  CHECK_EQF(id * x, x);
  CHECK_EQF(s * x, vec2(10, 40));
  CHECK_EQF(t * vec3(x, 1), vec3(11, 22, 1));
  CHECK_EQF(lengthSquared(m), 30);
  CHECK_EQF(determinant(m), -2);
  CHECK_EQF(determinant(s), 200);
  CHECK_EQF(determinant(mat2(vec2(1, 2), vec2(3, 6))), 0);
  CHECK_EQF(determinant(scale(vec3(2, 3, 5))), 30);
  CHECK_EQF(determinant(scale(vec4(2, 3, 5, 7))), 210);
  CHECK_EQF(replaceColumn<0>(m, x), mat2(vec2(1, 2), vec2(3, 4)));
  CHECK_EQF(replaceColumn<1>(m, x), mat2(vec2(1, 2), vec2(1, 2)));
  CHECK_EQF(solve(m, m * x), x);
  CHECK_EQF(solve(id, id * x), x);
  CHECK_EQF(solve(s, s * x), x);
  CHECK_EQF(solve(m3, m3 * x3), x3);
  CHECK_EQF(solve(m4, m4 * x4), x4);
  CHECK_EQF(inverse(m) * m, m.identity());
  CHECK_EQF(inverse(m3) * m3, m3.identity());
  CHECK_EQF(inverse(m4) * m4, m4.identity());
};

}  // namespace pine