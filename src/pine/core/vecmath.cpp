#include <pine/core/sampling.h>
#include <pine/core/vecmath.h>
#include <pine/core/context.h>

namespace pine {
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
      .member("x", &vec2i::x)
      .member("y", &vec2i::y)
      .method("[]", overloaded<int>(&vec2i::operator[]));
  ctx.type<vec2>()
      .ctor<float, float>()
      .ctor_variant<vec2i, float>()
      .member("x", &vec2::x)
      .member("y", &vec2::y)
      .method("[]", overloaded<int>(&vec2::operator[]));
  ctx.type<vec3i>()
      .ctor<int>()
      .ctor<int, int, int>()
      .member("x", &vec3i::x)
      .member("y", &vec3i::y)
      .member("z", &vec3i::z)
      .method("[]", overloaded<int>(&vec3i::operator[]));
  ctx.type<vec3>()
      .ctor<float, float, float>()
      .ctor_variant<vec3i, float>()
      .member("x", &vec3::x)
      .member("y", &vec3::y)
      .member("z", &vec3::z)
      .method("[]", overloaded<int>(&vec3::operator[]));
  ctx.type<vec4>()
      .ctor<float, float, float, float>()
      .ctor_variant<float>()
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
  ctx("pow") = overloaded<vec2, float>(pow<float>);
  ctx("pow") = overloaded<vec3, float>(pow<float>);
  ctx("pow") = overloaded<vec2, vec2>(pow<float>);
  ctx("pow") = overloaded<vec3, vec3>(pow<float>);
  ctx("coordinate_system") = overloaded<vec3>(coordinate_system);
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
  ctx("max_axis") = overloaded<vec3>(max_axis);
  ctx("min_axis") = overloaded<vec3>(min_axis);
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
