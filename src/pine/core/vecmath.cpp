#include <pine/core/sampling.h>
#include <pine/core/vecmath.h>
#include <pine/core/context.h>

namespace pine {

// The following two functions are by Francesco Mazzoli
// detailed in article "Speeding up atan2f by 50x": https://mazzo.li/posts/vectorized-atan2.html
inline float atan_fma_approximation(float x) {
  auto a1 = 0.99997726f;
  auto a3 = -0.33262347f;
  auto a5 = 0.19354346f;
  auto a7 = -0.11643287f;
  auto a9 = 0.05265332f;
  auto a11 = -0.01172120f;
  auto x_sq = x * x;
  return x * fmaf(x_sq, fmaf(x_sq, fmaf(x_sq, fmaf(x_sq, fmaf(x_sq, a11, a9), a7), a5), a3), a1);
}
float atan2_approx(float y, float x) {
  return psl::atan2(y, x);
  auto swap = fabs(x) < fabs(y);
  auto atan_input = (swap ? x : y) / (swap ? y : x);
  float res = atan_fma_approximation(atan_input);
  res = swap ? copysignf(Pi2, atan_input) - res : res;
  if (x < 0.0f) {
    res = copysignf(Pi, y) + res;
  }
  return res;
}

vec3 solve(mat3 m, vec3 b) {
  return inverse(m) * b;
  // {
  //   auto i = max_axis(abs(m[0]));
  //   if (i != 0) {
  //     psl::swap(m[0][0], m[0][i]);
  //     psl::swap(m[1][0], m[1][i]);
  //     psl::swap(m[2][0], m[2][i]);
  //     psl::swap(b[0], b[i]);
  //   }
  //   auto r1 = m[0][1] / m[0][0];
  //   auto r2 = m[0][2] / m[0][0];
  //   m[0][1] = 0;
  //   m[0][2] = 0;
  //   m[1][1] -= m[1][0] * r1;
  //   m[2][1] -= m[2][0] * r1;
  //   m[1][2] -= m[1][0] * r2;
  //   m[2][2] -= m[2][0] * r2;
  //   b[1] -= b[0] * r1;
  //   b[2] -= b[0] * r2;
  // }
  // {
  //   if (psl::abs(m[1][1]) < psl::abs(m[1][2])) {
  //     psl::swap(m[1][1], m[1][2]);
  //     psl::swap(m[2][1], m[2][2]);
  //     psl::swap(b[1], b[2]);
  //   }
  //   auto r2 = m[1][2] / m[1][1];
  //   m[1][2] = 0;
  //   m[2][2] -= m[2][1] * r2;
  //   b[2] -= b[1] * r2;
  // }
  // {
  //   b[1] -= b[2] * m[2][1] / m[2][2];
  //   b[0] -= b[2] * m[2][0] / m[2][2];
  //   b[0] -= b[1] * m[1][0] / m[1][1];
  // }

  // b[0] /= m[0][0];
  // b[1] /= m[1][1];
  // b[2] /= m[2][2];
  // return b;
}
mat3 inverse(const mat3& m) {
  mat3 r;

  float det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) +
              m[1][0] * (m[2][1] * m[0][2] - m[0][1] * m[2][2]) +
              m[2][0] * (m[0][1] * m[1][2] - m[1][1] * m[0][2]);
  if (det == 0)
    return r;

  r[0][0] = m[1][1] * m[2][2] - m[2][1] * m[1][2];
  r[0][1] = m[2][1] * m[0][2] - m[0][1] * m[2][2];
  r[0][2] = m[0][1] * m[1][2] - m[1][1] * m[0][2];

  r[1][0] = m[1][2] * m[2][0] - m[2][2] * m[1][0];
  r[1][1] = m[2][2] * m[0][0] - m[0][2] * m[2][0];
  r[1][2] = m[0][2] * m[1][0] - m[1][2] * m[0][0];

  r[2][0] = m[1][0] * m[2][1] - m[2][0] * m[1][1];
  r[2][1] = m[2][0] * m[0][1] - m[0][0] * m[2][1];
  r[2][2] = m[0][0] * m[1][1] - m[1][0] * m[0][1];

  return r / det;
}
mat4 inverse(const mat4& m) {
  mat4 r;
  float det = 0;
  for (int i = 0; i < 4; i++)
    det += (m[(1 + i) % 4][0] *
                (m[(2 + i) % 4][1] * m[(3 + i) % 4][2] - m[(3 + i) % 4][1] * m[(2 + i) % 4][2]) +
            m[(2 + i) % 4][0] *
                (m[(3 + i) % 4][1] * m[(1 + i) % 4][2] - m[(1 + i) % 4][1] * m[(3 + i) % 4][2]) +
            m[(3 + i) % 4][0] *
                (m[(1 + i) % 4][1] * m[(2 + i) % 4][2] - m[(2 + i) % 4][1] * m[(1 + i) % 4][2])) *
           m[i % 4][3] * (i % 2 ? -1 : 1);
  if (det == 0)
    return r;

  for (int v = 0; v < 4; v++)
    for (int i = 0; i < 4; i++)
      r[v][i] = (m[(1 + i) % 4][(1 + v) % 4] *
                     (m[(2 + i) % 4][(2 + v) % 4] * m[(3 + i) % 4][(3 + v) % 4] -
                      m[(3 + i) % 4][(2 + v) % 4] * m[(2 + i) % 4][(3 + v) % 4]) +
                 m[(2 + i) % 4][(1 + v) % 4] *
                     (m[(3 + i) % 4][(2 + v) % 4] * m[(1 + i) % 4][(3 + v) % 4] -
                      m[(1 + i) % 4][(2 + v) % 4] * m[(3 + i) % 4][(3 + v) % 4]) +
                 m[(3 + i) % 4][(1 + v) % 4] *
                     (m[(1 + i) % 4][(2 + v) % 4] * m[(2 + i) % 4][(3 + v) % 4] -
                      m[(2 + i) % 4][(2 + v) % 4] * m[(1 + i) % 4][(3 + v) % 4])) *
                ((v + i) % 2 ? 1 : -1);
  return r / det;
}

void vecmath_context(Context& ctx) {
  ctx.type<vec2i, Context::Complex>("vec2i");
  ctx.type<vec2, Context::Complex>("vec2");
  ctx.type<vec3i, Context::Complex>("vec3i");
  ctx.type<vec3, Context::Complex>("vec3");
  ctx.type<vec4, Context::Complex>("vec4");
  ctx.type<mat2>("mat2");
  ctx.type<mat3>("mat3");
  ctx.type<mat4>("mat4");

  ctx.type<vec2i>()
      .ctor<int>()
      .ctor<int, int>()
      .ctor_variant_explicit<vec2>()
      .member("x", &vec2i::x)
      .member("y", &vec2i::y)
      .method("[]", overloaded<int>(&vec2i::operator[]));
  ctx.type<vec2>()
      .ctor<float, float>()
      .ctor_variant<vec2i, int, float>()
      .member("x", &vec2::x)
      .member("y", &vec2::y)
      .method("[]", overloaded<int>(&vec2::operator[]));
  ctx.type<vec3i>()
      .ctor<int>()
      .ctor<int, int, int>()
      .ctor_variant_explicit<vec3>()
      .member("x", &vec3i::x)
      .member("y", &vec3i::y)
      .member("z", &vec3i::z)
      .method("[]", overloaded<int>(&vec3i::operator[]));
  ctx.type<vec3>()
      .ctor<float, float, float>()
      .ctor_variant<vec3i, int, float>()
      .member("x", &vec3::x)
      .member("y", &vec3::y)
      .member("z", &vec3::z)
      .method("[]", overloaded<int>(&vec3::operator[]));
  ctx.type<vec4>()
      .ctor<float, float, float, float>()
      .ctor_variant<int, float>()
      .member("x", &vec4::x)
      .member("y", &vec4::y)
      .member("z", &vec4::z)
      .member("w", &vec4::w)
      .method("[]", overloaded<int>(&vec4::operator[]));
  ctx.type<mat2>().ctor<vec2, vec2>().method("[]", overloaded<int>(&mat2::operator[]));
  ctx.type<mat3>().ctor<vec3, vec3, vec3>().ctor<mat4>().method("[]",
                                                                overloaded<int>(&mat3::operator[]));
  ctx.type<mat4>().ctor<vec4, vec4, vec4, vec4>().method("[]", overloaded<int>(&mat4::operator[]));
  ctx("*") = +[](const mat3& a, vec3 b) { return a * b; };
  ctx("*") = +[](const mat3& a, const mat3& b) { return a * b; };
  ctx("*") = +[](const mat4& a, vec4 b) { return a * b; };
  ctx("*") = +[](const mat4& a, const mat4& b) { return a * b; };
  ctx("*") = overloads_set<Overloads<vec2i, vec3i>, Overloads<int>>(psl::mul_);
  ctx("/") = overloads_set<Overloads<vec2i, vec3i>, Overloads<int>>(psl::div_);
  ctx("*") = overloads_set<Overloads<int>, Overloads<vec2i, vec3i>>(psl::mul_);
  ctx("/") = overloads_set<Overloads<int>, Overloads<vec2i, vec3i>>(psl::div_);
  ctx("*") = overloads_set<Overloads<vec2, vec3, vec4>, Overloads<float>>(psl::mul_);
  ctx("/") = overloads_set<Overloads<vec2, vec3, vec4>, Overloads<float>>(psl::div_);
  ctx("*") = overloads_set<Overloads<int, float>, Overloads<vec2, vec3, vec4>>(psl::mul_);
  ctx("/") = overloads_set<Overloads<int, float>, Overloads<vec2, vec3, vec4>>(psl::div_);
  ctx("*=") = overloads_set<Overloads<vec2i, vec3i>, Overloads<int>>(psl::mule_);
  ctx("/=") = overloads_set<Overloads<vec2i, vec3i>, Overloads<int>>(psl::dive_);
  ctx("*=") = overloads_set<Overloads<vec2, vec3, vec4>, Overloads<int, float>>(psl::mule_);
  ctx("/=") = overloads_set<Overloads<vec2, vec3, vec4>, Overloads<int, float>>(psl::dive_);
  ctx("normalize") = normalize<vec2>;
  ctx("normalize") = normalize<vec3>;
  ctx("length") = length<vec2>;
  ctx("length") = length<vec3>;
  ctx("distance") = distance<vec3>;
  ctx("distance") = distance<vec3>;
  ctx("dot") = overloaded<vec2, vec2>(dot<float>);
  ctx("dot") = overloaded<vec3, vec3>(dot<float>);
  ctx("cross") = overloaded<vec3, vec3>(dot<float>);
  ctx("fract") = overloaded<vec2>(fract<float>);
  ctx("fract") = overloaded<vec3>(fract<float>);
  ctx("floor") = overloaded<vec2>(floor<float>);
  ctx("floor") = overloaded<vec3>(floor<float>);
  ctx("ceil") = overloaded<vec2>(ceil<float>);
  ctx("ceil") = overloaded<vec3>(ceil<float>);
  ctx("sqrt") = overloaded<vec2>(sqrt<float>);
  ctx("sqrt") = overloaded<vec3>(sqrt<float>);
  ctx("exp") = overloaded<vec2>(exp<float>);
  ctx("exp") = overloaded<vec3>(exp<float>);
  ctx("abs") = overloaded<vec2>(abs<float>);
  ctx("abs") = overloaded<vec3>(abs<float>);
  ctx("min") = overloaded<vec2, vec2>(min<float>);
  ctx("min") = overloaded<vec3, vec3>(min<float>);
  ctx("max") = overloaded<vec2, vec2>(max<float>);
  ctx("max") = overloaded<vec3, vec3>(max<float>);
  ctx("clamp") = overloaded<vec2, vec2, vec2>(clamp<float>);
  ctx("clamp") = overloaded<vec3, vec3, vec3>(clamp<float>);
  ctx("lerp") = overloaded<vec2, vec2, vec2>(lerp<float>);
  ctx("lerp") = overloaded<vec3, vec3, vec3>(lerp<float>);
  ctx("pow") = overloaded<vec2, float>(pow<float>);
  ctx("pow") = overloaded<vec3, float>(pow<float>);
  ctx("pow") = overloaded<vec2, vec2>(pow<float>);
  ctx("pow") = overloaded<vec3, vec3>(pow<float>);
  ctx("coordinate_system") = overloaded_r<mat3, vec3>(coordinate_system);
  ctx("rotate_x") = rotate_x;
  ctx("rotate_y") = rotate_y;
  ctx("rotate_z") = rotate_z;
  ctx("rotate") = rotate;
  ctx("translate") = overloaded<vec3>(translate);
  ctx("translate") = overloaded<float, float, float>(translate);
  ctx("scale") = overloaded<vec3>(scale);
  ctx("scale") = overloaded<float, float, float>(scale);
  ctx("look_at") = look_at;
  ctx("identity3x3") = mat3::identity;
  ctx("identity4x4") = mat4::identity;
  ctx("transpose") = overloaded<const mat2&>(transpose<float>);
  ctx("transpose") = overloaded<const mat3&>(transpose<float>);
  ctx("transpose") = overloaded<const mat4&>(transpose<float>);
  ctx("determinant") = overloaded<const mat3&>(determinant);
  ctx("inverse") = overloaded<const mat2&>(inverse);
  ctx("inverse") = overloaded<const mat3&>(inverse);
  ctx("inverse") = overloaded<const mat4&>(inverse);
  ctx("max_axis") = overloaded<vec3i>(max_axis<int>);
  ctx("max_axis") = overloaded<vec3>(max_axis<float>);
  ctx("min_axis") = overloaded<vec3i>(min_axis<int>);
  ctx("min_axis") = overloaded<vec3>(min_axis<float>);
  ctx("spherical_to_cartesian") = spherical_to_cartesian;
  ctx("unit_square_to_cartesian") = unit_square_to_cartesian;
  ctx("cartesian_to_spherical") = cartesian_to_spherical;
  ctx("cartesian_to_unit_square") = cartesian_to_unit_square;
  ctx("sample_disk_polar") = sample_disk_polar;
  ctx("sample_disk_concentric") = sample_disk_concentric;
  ctx("cosine_weighted_hemisphere") = cosine_weighted_hemisphere;
  ctx("uniform_sphere") = uniform_sphere;
  ctx("inverse_uniform_sphere") = inverse_uniform_sphere;
  ctx("uniform_hemisphere") = uniform_hemisphere;
  ctx("inverse_uniform_hemisphere") = inverse_uniform_hemisphere;
}
}  // namespace pine
