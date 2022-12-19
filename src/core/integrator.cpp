#include <core/integrator.hpp>
#include <core/timer.hpp>

#include <iostream>

namespace pine {

void RayIntegrator::render() {
  auto timer = Timer();

  for (int sample = 0; sample < samplesPerPixel; ++sample) {
    for2D(scene.getCamera().imageSize(), [&](vec2i coordinate) {
      const auto cs = CameraSample(coordinate);
      const auto ray = scene.getCamera().generateRay(cs);
      image[coordinate] += radiance(ray).toRGB();
    });
  }

  std::cout << image.nPixels() * samplesPerPixel / timer.elapsed() / 1e+6f
            << " M samples/s\n";
  image /= samplesPerPixel;
  writeImageAsTGA(outputFileName, image);
}

Spectrum DebugIntegrator::radiance(Ray ray) const {
  auto it = Intersection();
  if (scene.intersect(ray, it))
    return Spectrum(it.n);
  else
    return Spectrum();
}

}  // namespace pine
