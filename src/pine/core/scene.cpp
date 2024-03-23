#include <pine/core/scene.h>
#include <pine/core/context.h>

#include <mutex>

namespace pine {

void Scene::add_material(psl::string name, psl::shared_ptr<Material> material) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock{mutex};
  materials[psl::move(name)] = psl::move(material);
}
void Scene::add_geometry(Shape shape, psl::shared_ptr<Material> material) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock{mutex};
  if (material->is<SubsurfaceMaterial>()) {
    auto sigma_s = material->as<SubsurfaceMaterial>().sigma_s;
    add_medium(Medium(HomogeneousMedium(shape, sigma_s / 8, sigma_s)));
  }
  geometries.push_back(psl::make_shared<Geometry>(psl::move(shape), material));
  if (geometries.back()->material->is<EmissiveMaterial>())
    add_light(AreaLight(geometries.back()));
}
void Scene::add_instancing(InstancedShape instancing) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock{mutex};
  instancings.push_back(psl::move(instancing));
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
}

AABB Scene::get_aabb() const {
  auto aabb = AABB{};
  for (const auto& g : geometries)
    aabb.extend(g->get_aabb());
  for (const auto& m : mediums)
    aabb.extend(m.get_aabb());
  return aabb;
}

void add_box(Scene& scene, mat4 m, psl::shared_ptr<Material> material) {
  scene.add_geometry(Rect::from_vertex({0, 0, 0}, {1, 0, 0}, {0, 1, 0}).apply(m), material);
  scene.add_geometry(Rect::from_vertex({0, 0, 1}, {1, 0, 1}, {0, 1, 1}).apply(m), material);
  scene.add_geometry(Rect::from_vertex({0, 0, 0}, {0, 1, 0}, {0, 0, 1}).apply(m), material);
  scene.add_geometry(Rect::from_vertex({1, 0, 0}, {1, 1, 0}, {1, 0, 1}).apply(m), material);
  scene.add_geometry(Rect::from_vertex({0, 0, 0}, {0, 0, 1}, {1, 0, 0}).apply(m), material);
  scene.add_geometry(Rect::from_vertex({0, 1, 0}, {0, 1, 1}, {1, 1, 0}).apply(m), material);
}

void scene_context(Context& ctx) {
  ctx.type<Scene>("Scene")
      .ctor<>()
      .member("camera", &Scene::camera)
      .method("add", overloaded<psl::string, psl::shared_ptr<Material>>(&Scene::add_material))
      .method("add", overloaded<psl::string, Material>(&Scene::add_material))
      .method("add", overloaded<Shape, psl::shared_ptr<Material>>(&Scene::add_geometry))
      .method("add", overloaded<Shape, Material>(&Scene::add_geometry))
      .method("add", overloaded<Shape, psl::string>(&Scene::add_geometry))
      .method("add", &Scene::add_instancing)
      .method("add", &Scene::add_light)
      .method("add", &Scene::add_medium)
      .method("set", &Scene::set_camera)
      .method("set", &Scene::set_env_light)
      .method("get_aabb", &Scene::get_aabb);
  ctx("add_box") = overloaded<Scene&, mat4, Material>(add_box);
  ctx("add_box") = overloaded<Scene&, mat4, psl::string>(add_box);
}

}  // namespace pine
