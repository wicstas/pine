#pragma once

#include <pine/core/geometry.h>
#include <pine/core/camera.h>
#include <pine/core/light.h>

#include <psl/memory.h>
#include <psl/vector.h>
#include <psl/map.h>

namespace pine {

struct Scene {
  void add_material(psl::string name, Material material);
  void add_geometry(Shape shape, psl::string material_name);
  void add_geometry(Shape shape, Material material);
  void add_light(Light light);
  void set_camera(Camera camera);
  void set_env_light(EnvironmentLight env_light);

  psl::shared_ptr<Material> find_material(psl::string_view name);
  void reset();
  AABB get_aabb() const;

  psl::map<psl::string, psl::shared_ptr<Material>> materials;
  psl::vector<psl::shared_ptr<Geometry>> geometries;
  psl::vector<Light> lights;
  psl::optional<EnvironmentLight> env_light;
  Camera camera;
};

void add_box(Scene& scene, mat4 m, Material material);
void add_box(Scene& scene, mat4 m, psl::string material_name);

void scene_context(Context& context);

}  // namespace pine
