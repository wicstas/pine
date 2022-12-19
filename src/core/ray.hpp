#pragma once
#include <core/math.hpp>

namespace pine {

struct Ray {
  vec3 operator()(float t) const { return o + t * d; }
  bool contains(float t) const { return between(t, tmin, tmax); }
  bool tooClose(float t) const { return t < tmin; }
  bool tooFar(float t) const { return t > tmax; }
  void updateTmax(float t) {
    DCHECK_LE(t, tmax);
    tmax = t;
  }

  vec3 o;
  vec3 d;
  float tmin = 0, tmax = Infinity;
};

}  // namespace pine
