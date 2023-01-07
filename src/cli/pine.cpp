#include <core/renderer.hpp>

using namespace pine;

int main() {
  auto scene = Scene();
  scene.setCamera(
      PinHoleCamera(vec2i(720, 480), vec3(0, 1, -4), vec3(0, 1, 0), Pi / 2));
  scene.addShape(Sphere(vec3(0, 1, 2), 1));
  scene.addShape(Plane(vec3(0, 0, 0), vec3(0, 1, 0)));

  auto renderer = RayRenderer(scene, 1, DebugIntegrator());

  renderer.render();

  saveImage("local/image.tga", scene.getCamera().getImage());
}