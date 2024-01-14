#include <pine/core/scene.h>

namespace pine {

void Scene::add_material(psl::string name, Material material) {
  materials[psl::move(name)] = psl::make_shared<Material>(psl::move(material));
}
void Scene::add_geometry(Shape shape, psl::string material_name) {
  geometries.push_back(psl::make_shared<Geometry>(psl::move(shape), find_material(material_name)));
  if (geometries.back()->material->is<EmissiveMaterial>())
    add_light(AreaLight(geometries.back()));
}
void Scene::add_geometry(Shape shape, Material material) {
  geometries.push_back(psl::make_shared<Geometry>(psl::move(shape),
                                                  psl::make_shared<Material>(psl::move(material))));
  if (geometries.back()->material->is<EmissiveMaterial>())
    add_light(AreaLight(geometries.back()));
}
void Scene::add_light(Light light) {
  lights.push_back(psl::move(light));
}
void Scene::set_camera(Camera camera) {
  this->camera = psl::move(camera);
}
void Scene::set_env_light(EnvironmentLight env_light) {
  this->env_light = psl::move(env_light);
}

psl::shared_ptr<Material> Scene::find_material(psl::string_view name) {
  if (const auto it = materials.find(name); it != materials.end())
    return it->second;
  else
    Fatal("Can't find material `", name, '`');
  PINE_UNREACHABLE;
}
void Scene::reset() {
  materials.clear();
  geometries.clear();
  geometries.clear();
  lights.clear();
  env_light = {};
  camera = {};
}

AABB Scene::get_aabb() const {
  auto aabb = AABB{};
  for (const auto& g : geometries)
    aabb.extend(g->get_aabb());
  return aabb;
}

void add_box(Scene& scene, mat4 m, Material material) {
  scene.add_geometry(Rect::from_vertex({0, 0, 0}, {1, 0, 0}, {0, 1, 0}).apply(m), material);
  scene.add_geometry(Rect::from_vertex({0, 0, 1}, {1, 0, 1}, {0, 1, 1}).apply(m), material);
  scene.add_geometry(Rect::from_vertex({0, 0, 0}, {0, 1, 0}, {0, 0, 1}).apply(m), material);
  scene.add_geometry(Rect::from_vertex({1, 0, 0}, {1, 1, 0}, {1, 0, 1}).apply(m), material);
  scene.add_geometry(Rect::from_vertex({0, 0, 0}, {0, 0, 1}, {1, 0, 0}).apply(m), material);
  scene.add_geometry(Rect::from_vertex({0, 1, 0}, {0, 1, 1}, {1, 1, 0}).apply(m), material);
}

void add_box(Scene& scene, mat4 m, psl::string material_name) {
  scene.add_geometry(Rect::from_vertex({0, 0, 0}, {1, 0, 0}, {0, 1, 0}).apply(m), material_name);
  scene.add_geometry(Rect::from_vertex({0, 0, 1}, {1, 0, 1}, {0, 1, 1}).apply(m), material_name);
  scene.add_geometry(Rect::from_vertex({0, 0, 0}, {0, 1, 0}, {0, 0, 1}).apply(m), material_name);
  scene.add_geometry(Rect::from_vertex({1, 0, 0}, {1, 1, 0}, {1, 0, 1}).apply(m), material_name);
  scene.add_geometry(Rect::from_vertex({0, 0, 0}, {0, 0, 1}, {1, 0, 0}).apply(m), material_name);
  scene.add_geometry(Rect::from_vertex({0, 1, 0}, {0, 1, 1}, {1, 1, 0}).apply(m), material_name);
}

}  // namespace pine
