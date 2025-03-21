#pragma once
#include <pine/core/integrator.h>
#include <pine/core/scene.h>

namespace pine {

class OpenGLPathIntegrator {
 public:
  void render(Scene& scene);
};

}  // namespace pine
