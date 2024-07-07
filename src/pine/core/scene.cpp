#include <pine/core/scene.h>
#include <pine/core/context.h>

#include <mutex>

namespace pine {

psl::shared_ptr<pine::Material> Scene::add_material(psl::string name,
                                                    psl::shared_ptr<Material> material) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock{mutex};
  return materials[MOVE(name)] = MOVE(material);
}
psl::shared_ptr<pine::Geometry> Scene::add_geometry(Shape shape,
                                                    psl::shared_ptr<Material> material) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock{mutex};
  geometries.push_back(psl::make_shared<Geometry>(MOVE(shape), material));
  if (geometries.back()->material->is<EmissiveMaterial>())
    add_light(AreaLight(geometries.back()));
  return geometries.back();
}
InstancedShape& Scene::add_instancing(InstancedShape instancing) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock{mutex};
  instancings.push_back(MOVE(instancing));
  return instancings.back();
}
Light& Scene::add_light(Light light) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock{mutex};
  lights.push_back(MOVE(light));
  return lights.back();
}
Medium& Scene::add_medium(Medium medium) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock{mutex};
  mediums.push_back(MOVE(medium));
  return mediums.back();
}
Camera& Scene::set_camera(Camera camera) {
  return this->camera = MOVE(camera);
}
EnvironmentLight& Scene::set_env_light(EnvironmentLight env_light) {
  return *(this->env_light = MOVE(env_light));
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

void scene_context(Context& ctx) {
  ctx.type<Scene>("Scene")
      .ctor<>()
      .member<&Scene::camera>("camera")
      .method<overloaded<psl::string, psl::shared_ptr<Material>>(&Scene::add_material)>("add")
      .method<overloaded<psl::string, Material>(&Scene::add_material)>("add")
      .method<overloaded<Shape, psl::shared_ptr<Material>>(&Scene::add_geometry)>("add")
      .method<overloaded<Shape, Material>(&Scene::add_geometry)>("add")
      .method<overloaded<Shape, psl::string>(&Scene::add_geometry)>("add")
      .method<&Scene::add_instancing>("add")
      .method<&Scene::add_light>("add")
      .method<&Scene::add_medium>("add")
      .method<&Scene::set_camera>("set")
      .method<&Scene::set_env_light>("set")
      .method<&Scene::get_aabb>("get_aabb");
}

}  // namespace pine
