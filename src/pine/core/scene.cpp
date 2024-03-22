#include <pine/core/scene.h>
#include <pine/core/context.h>

#include <mutex>

namespace pine {

void Scene::add_material(psl::string name, Material material) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock{mutex};
  materials[psl::move(name)] = psl::make_shared<Material>(psl::move(material));
}
int Scene::add_geometry(Shape shape, psl::string material_name) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock{mutex};
  const auto& material = find_material(material_name);
  if (material->is<SubsurfaceMaterial>())
    add_medium(
        Medium(HomogeneousMedium(shape, vec3(0.0f), material->as<SubsurfaceMaterial>().sigma_s)));
  geometries.push_back(psl::make_shared<Geometry>(psl::move(shape), material));
  if (geometries.back()->material->is<EmissiveMaterial>())
    add_light(AreaLight(geometries.back()));
  return geometries.size() - 1;
}
int Scene::add_geometry(Shape shape, Material material) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock{mutex};
  if (material.is<SubsurfaceMaterial>()) {
    add_medium(
        Medium(HomogeneousMedium(shape, vec3(0.0f), material.as<SubsurfaceMaterial>().sigma_s)));
  }
  geometries.push_back(psl::make_shared<Geometry>(psl::move(shape),
                                                  psl::make_shared<Material>(psl::move(material))));
  if (geometries.back()->material->is<EmissiveMaterial>())
    add_light(AreaLight(geometries.back()));
  return geometries.size() - 1;
}
void Scene::add_light(Light light) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock{mutex};
  lights.push_back(psl::move(light));
}
void Scene::add_medium(Medium medium) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock{mutex};
  mediums.push_back(psl::move(medium));
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
  for (const auto& m : mediums)
    aabb.extend(m.get_aabb());
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

void scene_context(Context& ctx) {
  ctx.type<Scene>("Scene")
      .ctor<>()
      .member("camera", &Scene::camera)
      .method("add", &Scene::add_material)
      .method("add", overloaded<Shape, psl::string>(&Scene::add_geometry))
      .method("add", overloaded<Shape, Material>(&Scene::add_geometry))
      .method("add", &Scene::add_light)
      .method("add", &Scene::add_medium)
      .method("set", &Scene::set_camera)
      .method("set", &Scene::set_env_light)
      .method("get_aabb", &Scene::get_aabb)
      .method("reset", &Scene::reset);
  ctx("add_box") = overloaded<Scene&, mat4, Material>(add_box);
  ctx("add_box") = overloaded<Scene&, mat4, psl::string>(add_box);
}

}  // namespace pine
