#pragma once
#include <pine/core/integrator.h>
#include <pine/core/scene.h>

namespace pine {

class OpenGLPathIntegrator {
 public:
  void add(Mesh mesh) {
    mesh.assign_normals_and_texcoords();
    meshes.push_back(MOVE(mesh));
  }

  void render();

 private:
  psl::vector<Mesh> meshes;
};

}  // namespace pine
