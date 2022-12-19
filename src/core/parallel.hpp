#pragma once
#include <core/math.hpp>

namespace pine {

void for2D(vec2i size, auto f) {
  for (int y = 0; y < size[1]; y++)
    for (int x = 0; x < size[0]; x++) f(vec2i(x, y));
}

}  // namespace pine
