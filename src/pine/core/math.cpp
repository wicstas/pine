#include <pine/core/context.h>
#include <pine/core/math.h>

namespace pine {

void math_context(Context& ctx) {
  ctx.var("Pi") = Pi;
  ctx.var("E") = psl::E;
  ctx("min") =  led<psl::min<int>>;
  ctx("min") =  led<psl::min<float>>;
  ctx("min") =  led<psl::min<int, int>>;
  ctx("min") =  led<psl::min<float, float>>;
  ctx("max") =  led<psl::max<int>>;
  ctx("max") =  led<psl::max<float>>;
  ctx("max") =  led<psl::max<int, int>>;
  ctx("max") =  led<psl::max<float, float>>;
  ctx("abs") =  led<psl::abs<int>>;
  ctx("abs") =  led<psl::abs<float>>;
  ctx("clamp") =  led<psl::clamp<int>>;
  ctx("clamp") =  led<psl::clamp<float>>;
  ctx("lerp") =  led<psl::lerp<float, float>>;
  ctx("sqr") =  led<psl::sqr<int>>;
  ctx("sqr") =  led<psl::sqr<float>>;
  ctx("sqrt") =  led<psl::sqrt<float>>;
  ctx("floor") =  led<psl::floor<float>>;
  ctx("ceil") =  led<psl::ceil<float>>;
  ctx("powi") =  led<psl::powi<int>>;
  ctx("pow") =  led<psl::pow<float>>;
  ctx("exp") =  led<psl::exp<float>>;
  ctx("log2") =  led<psl::log2<float>>;
  ctx("log") =  led<psl::log<float>>;
  ctx("log10") =  led<psl::log10<float>>;
  ctx("sin") =  led<psl::sin<float>>;
  ctx("cos") =  led<psl::cos<float>>;
  ctx("tan") =  led<psl::tan<float>>;
  ctx("acos") =  led<psl::acos<float>>;
  ctx("asin") =  led<psl::asin<float>>;
  ctx("atan2") =  led<psl::atan2<float>>;
  ctx("sinh") =  led<overloaded_r<float, float>(std::sinh)>;
  ctx("cosh") =  led<overloaded_r<float, float>(std::cosh)>;
  ctx("tanh") =  led<overloaded_r<float, float>(std::tanh)>;

}

}  // namespace pine
