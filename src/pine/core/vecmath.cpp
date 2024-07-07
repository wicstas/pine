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
  float abs_y = std::fabs(y) + 1e-10f;
  float r = (x - std::copysign(abs_y, x)) / (abs_y + std::fabs(x));
  float angle = M_PI / 2.f - std::copysign(M_PI / 4.f, x);

  angle += (0.1963f * r * r - 0.9817f) * r;
  return std::copysign(angle, y);
}
// float atan2_approx(float y, float x) {
//   auto swap = fabs(x) < fabs(y);
//   auto atan_input = (swap ? x : y) / (swap ? y : x);
//   float res = atan_fma_approximation(atan_input);
//   res = swap ? copysignf(Pi2, atan_input) - res : res;
//   if (x < 0.0f) {
//     res = copysignf(Pi, y) + res;
//   }
//   return res;
// }

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
  ctx.type<vec2i, Context::Complex>("vec2i").layout<int, int>();
  ctx.type<vec2, Context::Complex>("vec2").layout<float, float>();
  ctx.type<vec3i, Context::Complex>("vec3i").layout<int, int, int>();
  ctx.type<vec3, Context::Complex>("vec3").layout<float, float, float>();
  ctx.type<vec3u8, Context::Complex>("vec3u8").layout<uint8_t, uint8_t, uint8_t>();
  ctx.type<vec4i, Context::Complex>("vec4i").layout<int, int, int, int>();
  ctx.type<vec4, Context::Complex>("vec4").layout<float, float, float, float>();
  ctx.type<mat2>("mat2").layout<vec2, vec2>();
  ctx.type<mat3>("mat3").layout<vec3, vec3, vec3>();
  ctx.type<mat4>("mat4").layout<vec4, vec4, vec4, vec4>();

  ctx.type<psl::string>()
      .ctor_variant<[](const auto& x) { return to_string(x); }, vec2i, vec2, vec3i, vec3, vec3u8,
                    vec4i, vec4, mat2, mat3, mat4>();

  ctx.type<vec2i>()
      .ctor<int>()
      .ctor<int, int>()
      .ctor_variant_explicit<vec2>()
      .member<&vec2i::x>("x")
      .member<&vec2i::y>("y");
  ctx.type<vec2>()
      .ctor<float, float>()
      .ctor_variant<vec2i, float>()
      .member<&vec2::x>("x")
      .member<&vec2::y>("y")
      .method<overloaded(&vec2::operator[])>("[]");
  ctx.type<vec3i>()
      .ctor<int>()
      .ctor<int, int, int>()
      .ctor_variant_explicit<vec3>()
      .member<&vec3i::x>("x")
      .member<&vec3i::y>("y")
      .member<&vec3i::z>("z")
      .method<overloaded(&vec3i::operator[])>("[]");
  ctx.type<vec3>()
      .ctor<float, float, float>()
      .ctor_variant<vec3i, float>()
      .member<&vec3::x>("x")
      .member<&vec3::y>("y")
      .member<&vec3::z>("z")
      .method<overloaded(&vec3::operator[])>("[]");
  ctx.type<vec4i>()
      .ctor<int, int, int, int>()
      .ctor_variant_explicit<int>()
      .member<&vec4i::x>("x")
      .member<&vec4i::y>("y")
      .member<&vec4i::z>("z")
      .member<&vec4i::w>("w");
  ctx.type<vec4>()
      .ctor<float, float, float, float>()
      .ctor_variant<float>()
      .member<&vec4::x>("x")
      .member<&vec4::y>("y")
      .member<&vec4::z>("z")
      .member<&vec4::w>("w")
      .method<overloaded(&vec4::operator[])>("[]");
  ctx.type<mat2>().ctor<vec2, vec2>().method<overloaded<int>(&mat2::operator[])>("[]");
  ctx.type<mat3>().ctor<vec3, vec3, vec3>().ctor<mat4>().method<overloaded<int>(&mat3::operator[])>(
      "[]");
  ctx.type<mat4>().ctor<vec4, vec4, vec4, vec4>().method<overloaded<int>(&mat4::operator[])>("[]");
  ctx("+") = [](const mat2& a, const mat2& b) { return a + b; };
  ctx("+") = [](const mat3& a, const mat3& b) { return a + b; };
  ctx("+") = [](const mat4& a, const mat4& b) { return a + b; };
  ctx("+=") = [](mat2& a, const mat2& b) -> decltype(auto) { return a += b; };
  ctx("+=") = [](mat3& a, const mat3& b) -> decltype(auto) { return a += b; };
  ctx("+=") = [](mat4& a, const mat4& b) -> decltype(auto) { return a += b; };
  ctx("*") = [](const mat2& a, vec2 b) { return a * b; };
  ctx("*") = [](const mat3& a, vec3 b) { return a * b; };
  ctx("*") = [](const mat4& a, vec4 b) { return a * b; };
  ctx("*") = [](const mat2& a, const mat2& b) { return a * b; };
  ctx("*") = [](const mat3& a, const mat3& b) { return a * b; };
  ctx("*") = [](const mat4& a, const mat4& b) { return a * b; };
  ctx("*") = overloads_set<psl::mul_, TypeSet<vec2i, vec3i>, TypeSet<int>>();
  ctx("/") = overloads_set<psl::div_, TypeSet<vec2i, vec3i>, TypeSet<int>>();
  ctx("*") = overloads_set<psl::mul_, TypeSet<int>, TypeSet<vec2i, vec3i>>();
  ctx("/") = overloads_set<psl::div_, TypeSet<int>, TypeSet<vec2i, vec3i>>();
  ctx("*") = overloads_set<psl::mul_, TypeSet<vec2, vec3, vec4>, TypeSet<float>>();
  ctx("/") = overloads_set<psl::div_, TypeSet<vec2, vec3, vec4>, TypeSet<float>>();
  ctx("*") = overloads_set<psl::mul_, TypeSet<int, float>, TypeSet<vec2, vec3, vec4>>();
  ctx("/") = overloads_set<psl::div_, TypeSet<int, float>, TypeSet<vec2, vec3, vec4>>();
  ctx("*=") = overloads_set<psl::mule_, TypeSet<vec2i&, vec3i&>, TypeSet<int>>();
  ctx("/=") = overloads_set<psl::dive_, TypeSet<vec2i&, vec3i&>, TypeSet<int>>();
  ctx("*=") = overloads_set<psl::mule_, TypeSet<vec2&, vec3&, vec4&>, TypeSet<int, float>>();
  ctx("/=") = overloads_set<psl::dive_, TypeSet<vec2&, vec3&, vec4&>, TypeSet<int, float>>();
  ctx("normalize") = led<normalize<vec2>>;
  ctx("normalize") = led<normalize<vec3>>;
  ctx("length") = led<length<vec2>>;
  ctx("length") = led<length<vec3>>;
  ctx("distance") = led<distance<vec3>>;
  ctx("distance") = led<distance<vec3>>;
  ctx("dot") = led<overloaded<vec2, vec2>(dot<float>)>;
  ctx("dot") = led<overloaded<vec3, vec3>(dot<float>)>;
  ctx("sqr") = led<sqr<vec2>>;
  ctx("sqr") = led<sqr<vec3>>;
  ctx("sqr") = led<sqr<vec4>>;
  ctx("sum") = led<overloaded<vec2>(sum<float>)>;
  ctx("sum") = led<overloaded<vec3>(sum<float>)>;
  ctx("sum") = led<overloaded<vec4>(sum<float>)>;
  ctx("cross") = led<overloaded<vec3, vec3>(dot<float>)>;
  ctx("fract") = led<overloaded<vec2>(fract<float>)>;
  ctx("fract") = led<overloaded<vec3>(fract<float>)>;
  ctx("floor") = led<overloaded<vec2>(floor<float>)>;
  ctx("floor") = led<overloaded<vec3>(floor<float>)>;
  ctx("ceil") = led<overloaded<vec2>(ceil<float>)>;
  ctx("ceil") = led<overloaded<vec3>(ceil<float>)>;
  ctx("sqrt") = led<overloaded<vec2>(sqrt<float>)>;
  ctx("sqrt") = led<overloaded<vec3>(sqrt<float>)>;
  ctx("exp") = led<overloaded<vec2>(exp<float>)>;
  ctx("exp") = led<overloaded<vec3>(exp<float>)>;
  ctx("abs") = led<overloaded<vec2>(abs<float>)>;
  ctx("abs") = led<overloaded<vec3>(abs<float>)>;
  ctx("min") = led<overloaded<vec2, vec2>(min<float>)>;
  ctx("min") = led<overloaded<vec3, vec3>(min<float>)>;
  ctx("max") = led<overloaded<vec2, vec2>(max<float>)>;
  ctx("max") = led<overloaded<vec3, vec3>(max<float>)>;
  ctx("clamp") = led<overloaded<vec2, vec2, vec2>(clamp<float>)>;
  ctx("clamp") = led<overloaded<vec3, vec3, vec3>(clamp<float>)>;
  ctx("lerp") = led<overloaded<vec2, vec2, vec2>(lerp<float>)>;
  ctx("lerp") = led<overloaded<vec3, vec3, vec3>(lerp<float>)>;
  ctx("pow") = led<overloaded<vec2, float>(pow<float>)>;
  ctx("pow") = led<overloaded<vec3, float>(pow<float>)>;
  ctx("pow") = led<overloaded<vec2, vec2>(pow<float>)>;
  ctx("pow") = led<overloaded<vec3, vec3>(pow<float>)>;
  ctx("sin") = [](vec2 x) { return vec2(psl::sin(x.x), psl::sin(x.y)); };
  ctx("sin") = [](vec3 x) { return vec3(psl::sin(x.x), psl::sin(x.y), psl::sin(x.z)); };
  ctx("cos") = [](vec2 x) { return vec2(psl::cos(x.x), psl::cos(x.y)); };
  ctx("cos") = [](vec3 x) { return vec3(psl::cos(x.x), psl::cos(x.y), psl::cos(x.z)); };
  ctx("tanh") = [](vec2 x) { return vec2(std::tanh(x.x), std::tanh(x.y)); };
  ctx("coordinate_system") = led<overloaded_r<mat3, vec3>(coordinate_system)>;
  ctx("rotate_x") = led<rotate_x>;
  ctx("rotate_y") = led<rotate_y>;
  ctx("rotate_z") = led<rotate_z>;
  ctx("rotate") = led<rotate>;
  ctx("translate") = led<overloaded<vec3>(translate)>;
  ctx("translate") = led<overloaded<float, float, float>(translate)>;
  ctx("scale") = led<overloaded<vec3>(scale)>;
  ctx("scale") = led<overloaded<float, float, float>(scale)>;
  ctx("scale") = [](float s) { return scale(s, s, s); };
  ctx("look_at") = led<look_at>;
  ctx("identity3x3") = led<mat3::identity>;
  ctx("identity4x4") = led<mat4::identity>;
  ctx("transpose") = led<overloaded<const mat2&>(transpose<float>)>;
  ctx("transpose") = led<overloaded<const mat3&>(transpose<float>)>;
  ctx("transpose") = led<overloaded<const mat4&>(transpose<float>)>;
  ctx("determinant") = led<overloaded<const mat3&>(determinant)>;
  ctx("inverse") = led<overloaded<const mat2&>(inverse)>;
  ctx("inverse") = led<overloaded<const mat3&>(inverse)>;
  ctx("inverse") = led<overloaded<const mat4&>(inverse)>;
  ctx("solve") = led<solve>;
  ctx("max_axis") = led<overloaded<vec3i>(max_axis<int>)>;
  ctx("max_axis") = led<overloaded<vec3>(max_axis<float>)>;
  ctx("min_axis") = led<overloaded<vec3i>(min_axis<int>)>;
  ctx("min_axis") = led<overloaded<vec3>(min_axis<float>)>;
  ctx("max_value") = led<overloaded<vec3i>(max_value<int>)>;
  ctx("max_value") = led<overloaded<vec3>(max_value<float>)>;
  ctx("min_value") = led<overloaded<vec3i>(min_value<int>)>;
  ctx("min_value") = led<overloaded<vec3>(min_value<float>)>;
  ctx("spherical_to_cartesian") = led<overloaded_r<vec3, float, float>(spherical_to_cartesian)>;
  ctx("spherical_to_cartesian") =
      led<overloaded_r<vec3, float, float, float>(spherical_to_cartesian)>;
  ctx("unit_square_to_cartesian") = led<unit_square_to_cartesian>;
  ctx("cartesian_to_spherical") = led<cartesian_to_spherical>;
  ctx("cartesian_to_unit_square") = led<cartesian_to_unit_square>;
  ctx("sample_disk_polar") = led<sample_disk_polar>;
  ctx("sample_disk_concentric") = led<sample_disk_concentric>;
  ctx("cosine_weighted_hemisphere") = led<cosine_weighted_hemisphere>;
  ctx("uniform_sphere") = led<uniform_sphere>;
  ctx("inverse_uniform_sphere") = led<inverse_uniform_sphere>;
  ctx("uniform_hemisphere") = led<uniform_hemisphere>;
  ctx("inverse_uniform_hemisphere") = led<inverse_uniform_hemisphere>;
  ctx.var("X") = vec3(1, 0, 0);
  ctx.var("Y") = vec3(0, 1, 0);
  ctx.var("Z") = vec3(0, 0, 1);
}
}  // namespace pine
