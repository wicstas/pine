#pragma once
#include <pine/core/integrator.h>
#include <psl/array.h>

namespace pine {

class MicroRenderIntegrator : public Integrator {
 public:
  MicroRenderIntegrator(int point_count) : point_count(point_count) {}

  void render(Scene& scene) override;

 private:
  int point_count;

  using Disc = psl::Array<float, 10>;
  // struct  Disc {
  //   vec3 p;
  //   vec3 n;
  //   float r;
  //   vec3 color;
  // };
  psl::vector<Disc> discs;
};

}  // namespace pine
