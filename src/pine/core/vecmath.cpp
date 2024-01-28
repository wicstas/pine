#include <pine/core/vecmath.h>
#include <pine/core/context.h>

namespace pine {
void vecmath_context(Context& ctx) {
  ctx.type<vec2i, Context::Complex>("vec2i")
      .ctor<int>()
      .ctor<int, int>()
      .member("x", &vec2i::x)
      .member("y", &vec2i::y);
  ctx.type<vec3i, Context::Complex>("vec3i")
      .ctor<int>()
      .ctor<int, int, int>()
      .member("x", &vec3i::x)
      .member("y", &vec3i::y)
      .member("z", &vec3i::z);
  ctx.type<vec2, Context::Complex>("vec2")
      .ctor<float, float>()
      .ctor_variant<vec2i, int, float>()
      .member("x", &vec2::x)
      .member("y", &vec2::y);
  ctx.type<vec3, Context::Complex>("vec3")
      .ctor<float, float, float>()
      .ctor_variant<vec3i, int, float>()
      .member("x", &vec3::x)
      .member("y", &vec3::y)
      .member("z", &vec3::z);
  ctx.type<vec4, Context::Complex>("vec4")
      .ctor<float, float, float, float>()
      .ctor_variant<vec4i, int, float>()
      .member("x", &vec4::x)
      .member("y", &vec4::y)
      .member("z", &vec4::z)
      .member("w", &vec4::w);
  ctx.type<mat3>("mat3").ctor<vec3, vec3, vec3>().ctor<mat4>();
  ctx.type<mat4>("mat4").ctor<vec4, vec4, vec4, vec4>();
  ctx("*") = +[](const mat3& a, vec3 b) { return a * b; };
  ctx("*") = +[](const mat3& a, const mat3& b) { return a * b; };
  ctx("*") = +[](const mat4& a, vec4 b) { return a * b; };
  ctx("*") = +[](const mat4& a, const mat4& b) { return a * b; };
  ctx("translate") = overloaded<vec3>(translate);
  ctx("scale") = overloaded<vec3>(scale);
  ctx("look_at") = look_at;
  ctx("*") = overloads_set<Overloads<vec2i, vec3i>, Overloads<int>>(psl::mul_);
  ctx("/") = overloads_set<Overloads<vec2i, vec3i>, Overloads<int>>(psl::div_);
  ctx("*") = overloads_set<Overloads<int>, Overloads<vec2i, vec3i>>(psl::mul_);
  ctx("/") = overloads_set<Overloads<int>, Overloads<vec2i, vec3i>>(psl::div_);
  ctx("*") = overloads_set<Overloads<vec2, vec3, vec4>, Overloads<float>>(psl::mul_);
  ctx("/") = overloads_set<Overloads<vec2, vec3, vec4>, Overloads<float>>(psl::div_);
  ctx("*") = overloads_set<Overloads<float>, Overloads<vec2, vec3, vec4>>(psl::mul_);
  ctx("/") = overloads_set<Overloads<float>, Overloads<vec2, vec3, vec4>>(psl::div_);
  ctx("*=") = overloads_set<Overloads<vec2i, vec3i>, Overloads<int>>(psl::mule_);
  ctx("/=") = overloads_set<Overloads<vec2i, vec3i>, Overloads<int>>(psl::dive_);
  ctx("*=") = overloads_set<Overloads<vec2, vec3, vec4>, Overloads<float>>(psl::mule_);
  ctx("/=") = overloads_set<Overloads<vec2, vec3, vec4>, Overloads<float>>(psl::dive_);
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
  ctx("coordinate_system") = overloadedr<mat3, vec3>(coordinate_system);
  ctx("rotate_x") = rotate_x;
  ctx("rotate_y") = rotate_y;
  ctx("rotate_z") = rotate_z;
  ctx("rotate") = rotate;
  ctx("identity3x3") = mat3::identity;
  ctx("identity4x4") = mat4::identity;
}
}  // namespace pine
