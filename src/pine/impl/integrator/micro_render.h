#pragma once
#include <pine/core/integrator.h>
#include <psl/array.h>

namespace pine {

class MicroRenderIntegrator : public Integrator {
 public:
  MicroRenderIntegrator(int point_count) : point_count(psl::roundup2(point_count)) {}

  void render(Scene& scene) override;

 private:
  int point_count;
};

}  // namespace pine
