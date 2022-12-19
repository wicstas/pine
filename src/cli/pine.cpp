#include <core/integrator.hpp>

using namespace pine;

int main() {
  auto scene = Scene();
  scene.setCamera(PinHoleCamera(vec2i(720, 480)), lookAt(vec3(0, 1, -4), vec3(0, 1, 0)));
  scene.addShape(Sphere(vec3(0, 1, 2), 1));
  scene.addShape(Plane(vec3(0, 0, 0), vec3(0, 1, 0)));

  auto integrator = DebugIntegrator(scene, "local/image.tga", 1);

  integrator.render();
}