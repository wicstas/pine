#include <pine/core/context.h>
#include <pine/core/math.h>

namespace pine {

void math_context(Context &ctx) {
  ctx("Pi") = Pi;
  ctx("E") = psl::E;
  ctx("min") = psl::min<float>;
  ctx("max") = psl::max<float>;
  ctx("abs") = psl::abs<float>;
  ctx("clamp") = psl::clamp<float>;
  ctx("lerp") = psl::lerp<float, float>;
  ctx("sqr") = psl::sqr<float>;
  ctx("sqrt") = psl::sqrt<float>;
  ctx("floor") = psl::floor<float>;
  ctx("ceil") = psl::ceil<float>;
  ctx("powi") = psl::powi<int>;
  ctx("pow") = psl::pow<float>;
  ctx("exp") = psl::exp<float>;
  ctx("log2") = psl::log2<float>;
  ctx("log") = psl::log<float>;
  ctx("log10") = psl::log10<float>;
  ctx("sin") = psl::sin<float>;
  ctx("cos") = psl::cos<float>;
  ctx("tan") = psl::tan<float>;
  ctx("acos") = psl::acos<float>;
  ctx("asin") = psl::asin<float>;
  ctx("atan2") = psl::atan2<float>;
}

}  // namespace pine
