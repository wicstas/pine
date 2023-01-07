#pragma once
#include <core/parallel.hpp>
#include <core/scene.hpp>
#include <core/image.hpp>
#include <core/color.hpp>

#include <functional>

namespace pine {

class Renderer {
 public:
  Renderer(Scene& scene) : scene(scene) {}
  virtual ~Renderer() = default;

  virtual void render() = 0;

 protected:
  Scene& scene;
};

using Integrator = std::function<Spectrum(const Scene& scene, Ray)>;

class RayRenderer : public Renderer {
 public:
  RayRenderer(Scene& scene, int samplesPerPixel, Integrator integrator)
      : Renderer(scene),
        samplesPerPixel(samplesPerPixel),
        integrator(integrator) {}

  void render() override;

 protected:
  int samplesPerPixel;
  Integrator integrator;
};

struct DebugIntegrator {
  Spectrum operator()(const Scene& scene, Ray ray) const;
};

}  // namespace pine
