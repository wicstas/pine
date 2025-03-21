#include <pine/impl/integrator/path_opengl.cpp>
#include <pine/core/fileio.h>

using namespace pine;

Mesh gen_mesh(auto... vs) {
  auto mesh = Mesh();
  mesh.vertices.push_back(vs...);
  for (size_t i = 0; i < mesh.vertices.size() - 2; i++) mesh.indices.push_back({0, i + 1, i + 2});
  return mesh;
}

int main() {
  auto renderer = OpenGLPathIntegrator();

  // renderer.add(gen_mesh(vec3(0, 0, 0), vec3(1, 0, 0), vec3(1, 1, 0), vec3(0, 1, 0)));

  auto scene = load_scene("scenes/cbox.glb");
  renderer.render(scene);
}