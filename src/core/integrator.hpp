#pragma once
#include <core/parallel.hpp>
#include <core/scene.hpp>
#include <core/image.hpp>
#include <core/color.hpp>

namespace pine {

class Integrator {
 public:
  Integrator(Scene scene) : scene(std::move(scene)) {}
  ~Integrator() = default;

  virtual void render() = 0;

 protected:
  Scene scene;
};

class RayIntegrator : public Integrator {
 public:
  RayIntegrator(Scene scene_, std::string outputFileName, int samplesPerPixel)
      : Integrator(std::move(scene_)),
        image(scene.getCamera().imageSize()),
        outputFileName(outputFileName),
        samplesPerPixel(samplesPerPixel) {}

  void render() override;

 protected:
  virtual Spectrum radiance(Ray ray) const = 0;

  Image<vec3> image;
  std::string outputFileName;
  int samplesPerPixel;
};

class DebugIntegrator : public RayIntegrator {
 public:
  using RayIntegrator::RayIntegrator;

 private:
  Spectrum radiance(Ray ray) const override;
};

}  // namespace pine
