#include <pine/core/vecmath.h>

namespace pine {

/*
# Blackbody color datafile (bbr_color.txt)
# Mitchell Charity
# http://www.vendian.org/mncharity/dir3/blackbody/
# Version 2001-Jun-22
*/
// Range: 1000K - 10000K
constexpr int blackbody_color_constants_n = 182;
extern vec3 blackbody_color_constants[blackbody_color_constants_n];
inline vec3 blackbody(float k) {
  auto i = psl::lerp((k - 1000.0f) / (9000.0f), 0, blackbody_color_constants_n);
  i = psl::clamp(i, 0, blackbody_color_constants_n - 1);
  return blackbody_color_constants[int(i)];
}

}  // namespace pine