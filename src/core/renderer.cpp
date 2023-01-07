#include <core/renderer.hpp>
#include <core/logging.hpp>

namespace pine {

void RayRenderer::render() {
  for (int sample = 0; sample < samplesPerPixel; ++sample) {
    for2D(scene.getCamera().imageSize(), [&](vec2i coordinate) {
      const auto cs = CameraSample(coordinate);
      const auto ray = scene.getCamera().generateRay(cs);
      const auto color = integrator(scene, ray);
      scene.getCamera().recordColor(cs, color);
    });
  }

  scene.getCamera().getImage() /= samplesPerPixel;
}

Spectrum DebugIntegrator::operator()(const Scene& scene, Ray ray) const {
  auto it = Intersection();
  if (scene.intersect(ray, it))
    return Spectrum(it.n);
  else
    return Spectrum();
}

}  // namespace pine
