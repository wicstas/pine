#pragma once

#include <pine/core/geometry.h>
#include <pine/core/camera.h>
#include <pine/core/medium.h>
#include <pine/core/light.h>

#include <psl/memory.h>
#include <psl/vector.h>
#include <psl/map.h>

namespace pine {

struct Scene {
  psl::shared_ptr<pine::Material> add_material(psl::string name,
                                               psl::shared_ptr<Material> material);
  psl::shared_ptr<pine::Material> add_material(psl::string name, Material material) {
    return add_material(MOVE(name), psl::make_shared<Material>(MOVE(material)));
  }
  psl::shared_ptr<pine::Geometry> add_geometry(Shape shape, psl::shared_ptr<Material> material);
  psl::shared_ptr<pine::Geometry> add_geometry(Shape shape, Material material) {
    return add_geometry(MOVE(shape), psl::make_shared<Material>(MOVE(material)));
  }
  psl::shared_ptr<pine::Geometry> add_geometry(Shape shape, psl::string material_name) {
    return add_geometry(MOVE(shape), find_material(material_name));
  }
  InstancedShape& add_instancing(InstancedShape instancing);
  Light& add_light(Light light);
  Medium& add_medium(Medium medium);
  Camera& set_camera(Camera camera);
  EnvironmentLight& set_env_light(EnvironmentLight env_light);

  psl::shared_ptr<Material> find_material(psl::string_view name);
  AABB get_aabb() const;

  psl::map<psl::string, psl::shared_ptr<Material>> materials;
  psl::vector<psl::shared_ptr<Geometry>> geometries;
  psl::vector<InstancedShape> instancings;
  psl::vector<Light> lights;
  psl::vector<Medium> mediums;
  psl::optional<EnvironmentLight> env_light;
  Camera camera;
};

void add_box(Scene& scene, mat4 m, psl::shared_ptr<Material> material);
inline void add_box(Scene& scene, mat4 m, Material material) {
  add_box(scene, m, psl::make_shared<Material>(MOVE(material)));
}
inline void add_box(Scene& scene, mat4 m, psl::string material_name) {
  add_box(scene, m, scene.find_material(material_name));
}

void scene_context(Context& context);

}  // namespace pine
