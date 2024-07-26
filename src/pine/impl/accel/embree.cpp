#include <embree4/rtcore.h>
#include <pine/core/profiler.h>
#include <pine/core/scene.h>
#include <pine/impl/accel/embree.h>

namespace pine {

struct RayInteraction : RTCRayHit {
  SurfaceInteraction* it;
};

static void bounds_func(const RTCBoundsFunctionArguments* args) {
  const Shape& shape = *reinterpret_cast<const Shape*>(args->geometryUserPtr);
  RTCBounds* bounds_o = args->bounds_o;
  auto aabb = shape.get_aabb();
  bounds_o->lower_x = aabb.lower.x;
  bounds_o->lower_y = aabb.lower.y;
  bounds_o->lower_z = aabb.lower.z;
  bounds_o->upper_x = aabb.upper.x;
  bounds_o->upper_y = aabb.upper.y;
  bounds_o->upper_z = aabb.upper.z;
}

static void intersect_func(const RTCIntersectFunctionNArguments* args) {
  if (!args->valid[0]) return;

  const Shape& shape = *reinterpret_cast<const Shape*>(args->geometryUserPtr);
  auto& ray_ = reinterpret_cast<RayInteraction*>(args->rayhit)->ray;
  auto& hit_ = reinterpret_cast<RayInteraction*>(args->rayhit)->hit;
  auto& it = *reinterpret_cast<RayInteraction*>(args->rayhit)->it;

  auto ray = Ray(vec3(ray_.org_x, ray_.org_y, ray_.org_z), vec3(ray_.dir_x, ray_.dir_y, ray_.dir_z),
                 ray_.tnear, ray_.tfar);
  if (shape.intersect(ray, it)) {
    ray_.tfar = ray.tmax;
    hit_.geomID = args->geomID;
    hit_.primID = args->primID;
    args->valid[0] = -1;
  }
}
static void hit_func(const RTCOccludedFunctionNArguments* args) {
  if (!args->valid[0]) return;

  const Shape& shape = *reinterpret_cast<const Shape*>(args->geometryUserPtr);
  auto& ray_ = *reinterpret_cast<RTCRay*>(args->ray);

  auto ray = Ray(vec3(ray_.org_x, ray_.org_y, ray_.org_z), vec3(ray_.dir_x, ray_.dir_y, ray_.dir_z),
                 ray_.tnear, ray_.tfar);
  if (shape.hit(ray)) {
    ray_.tfar = -Infinity;
    args->valid[0] = -1;
  }
}
static void hit8_func(const RTCOccludedFunctionNArguments* args) {
  const Shape& shape = *reinterpret_cast<const Shape*>(args->geometryUserPtr);
  auto& ray_ = *reinterpret_cast<RTCRay8*>(args->ray);

  for (int i = 0; i < 8; i++) {
    if (args->valid[i]) {
      if (shape.hit(Ray(vec3(ray_.org_x[i], ray_.org_y[i], ray_.org_z[i]),
                        vec3(ray_.dir_x[i], ray_.dir_y[i], ray_.dir_z[i]), ray_.tnear[i],
                        ray_.tfar[i])))
        ray_.tfar[i] = -Infinity;
    }
  }
}
static void filter_func(const RTCFilterFunctionNArguments* args) {
  if (!args->valid[0]) return;

  //   const auto& geo = *reinterpret_cast<const Geometry*>(args->geometryUserPtr);
  //   args->valid[0] = 0;
}
static void build_geom(RTCDevice rtc_device, RTCScene rtc_scene, const Shape& shape,
                       uint32_t expected_index) {
  if (shape.is<Mesh>()) {
    auto mesh = shape.as<Mesh>();
    RTCGeometry geom = rtcNewGeometry(rtc_device, RTC_GEOMETRY_TYPE_TRIANGLE);
    rtcSetGeometryEnableFilterFunctionFromArguments(geom, true);

    float* vb = (float*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
                                                sizeof(vec3), mesh.vertices.size());
    psl::memcpy(vb, mesh.vertices.data(), mesh.vertices.byte_size());

    unsigned* ib = (unsigned*)rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(vec3u32), mesh.num_triangles());
    psl::memcpy(ib, mesh.indices.data(), mesh.indices.byte_size());
    rtcCommitGeometry(geom);
    CHECK_EQ(expected_index, rtcAttachGeometry(rtc_scene, geom));
    rtcReleaseGeometry(geom);
  } else {
    RTCGeometry geom = rtcNewGeometry(rtc_device, RTC_GEOMETRY_TYPE_USER);
    rtcSetGeometryEnableFilterFunctionFromArguments(geom, true);
    rtcSetGeometryUserPrimitiveCount(geom, 1);
    rtcSetGeometryUserData(geom, (void*)&shape);
    rtcSetGeometryBoundsFunction(geom, bounds_func, nullptr);
    rtcCommitGeometry(geom);
    CHECK_EQ(expected_index, rtcAttachGeometry(rtc_scene, geom));
    rtcReleaseGeometry(geom);
  }
}
void EmbreeAccel::build(const Scene* scene) {
  this->scene = scene;
  rtc_device =
      psl::opaque_shared_ptr(rtcNewDevice(nullptr), +[](RTCDevice ptr) { rtcReleaseDevice(ptr); });
  auto rtc_device = RTCDevice(this->rtc_device.get());
  rtcSetDeviceErrorFunction(
      rtc_device, +[](void*, enum RTCError, const char* str) { Fatal("[Embree]", str); }, nullptr);
  rtc_scene =
      psl::opaque_shared_ptr(rtcNewScene(rtc_device), +[](RTCScene ptr) { rtcReleaseScene(ptr); });
  auto rtc_scene = RTCScene(this->rtc_scene.get());
  rtcSetSceneFlags(rtc_scene, RTC_SCENE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS);
  rtcSetSceneBuildQuality(rtc_scene, RTC_BUILD_QUALITY_HIGH);

  for (uint32_t i = 0; i < scene->geometries.size(); i++) {
    build_geom(rtc_device, rtc_scene, scene->geometries[i]->shape, i);
    indices_to_instancing_index.push_back(uint32_t(-1));
    indices_to_instance_index.push_back(uint32_t(-1));
  }

  for (size_t i = 0; i < scene->instancings.size(); i++) {
    this->instancing_scenes.push_back(psl::opaque_shared_ptr(
        rtcNewScene(rtc_device), +[](RTCScene ptr) { rtcReleaseScene(ptr); }));
    auto instancing_scene = RTCScene(instancing_scenes.back().get());
    rtcSetSceneFlags(instancing_scene, RTC_SCENE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS);
    rtcSetSceneBuildQuality(instancing_scene, RTC_BUILD_QUALITY_HIGH);
    build_geom(rtc_device, instancing_scene, scene->instancings[i].shape, 0);
    rtcCommitScene(instancing_scene);

    for (size_t k = 0; k < scene->instancings[i].instances.size(); k++) {
      RTCGeometry geom = rtcNewGeometry(rtc_device, RTC_GEOMETRY_TYPE_INSTANCE);
      rtcSetGeometryEnableFilterFunctionFromArguments(geom, true);
      rtcSetGeometryInstancedScene(geom, instancing_scene);
      rtcSetGeometryTransform(geom, 0, RTCFormat::RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,
                              &scene->instancings[i].instances[k].transform[0][0]);
      rtcCommitGeometry(geom);
      CHECK_EQ(indices_to_instancing_index.size(), rtcAttachGeometry(rtc_scene, geom));
      indices_to_instancing_index.push_back(i);
      indices_to_instance_index.push_back(k);
      rtcReleaseGeometry(geom);
    }
  }
  rtcCommitScene(rtc_scene);
}
bool EmbreeAccel::hit(Ray ray) const {
  RTCRay ray_;
  ray_.org_x = ray.o.x;
  ray_.org_y = ray.o.y;
  ray_.org_z = ray.o.z;
  ray_.dir_x = ray.d.x;
  ray_.dir_y = ray.d.y;
  ray_.dir_z = ray.d.z;
  ray_.tnear = ray.tmin;
  ray_.tfar = ray.tmax;
  ray_.mask = -1;

  RTCOccludedArguments args;
  rtcInitOccludedArguments(&args);
  args.occluded = hit_func;
  args.filter = filter_func;
  args.feature_mask = RTCFeatureFlags(RTC_FEATURE_FLAG_TRIANGLE | RTC_FEATURE_FLAG_INSTANCE |
                                      RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS |
                                      RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_ARGUMENTS);
  rtcOccluded1(RTCScene(rtc_scene.get()), &ray_, &args);
  return ray_.tfar < 0;
}
uint8_t EmbreeAccel::hit8(psl::span<const Ray> rays) const {
  RTCRay8 ray_;
  for (int i = 0; i < 8; i++) {
    ray_.org_x[i] = rays[i].o.x;
    ray_.org_y[i] = rays[i].o.y;
    ray_.org_z[i] = rays[i].o.z;
    ray_.dir_x[i] = rays[i].d.x;
    ray_.dir_y[i] = rays[i].d.y;
    ray_.dir_z[i] = rays[i].d.z;
    ray_.tnear[i] = rays[i].tmin;
    ray_.tfar[i] = rays[i].tmax;
    ray_.mask[i] = -1;
  }

  RTCOccludedArguments args;
  rtcInitOccludedArguments(&args);
  args.occluded = hit8_func;
  args.filter = filter_func;
  args.feature_mask = RTCFeatureFlags(RTC_FEATURE_FLAG_TRIANGLE | RTC_FEATURE_FLAG_INSTANCE |
                                      RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS |
                                      RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_ARGUMENTS);
  int valid[8]{-1, -1, -1, -1, -1, -1, -1, -1};
  rtcOccluded8(valid, RTCScene(rtc_scene.get()), &ray_, &args);
  auto result = uint8_t(0);
  for (int i = 0; i < 8; i++)
    if (ray_.tfar[i] < 0) result |= 1 << i;

  return result;
}
bool EmbreeAccel::intersect(Ray& ray, SurfaceInteraction& it) const {
  RayInteraction rayhit;
  rayhit.ray.org_x = ray.o.x;
  rayhit.ray.org_y = ray.o.y;
  rayhit.ray.org_z = ray.o.z;
  rayhit.ray.dir_x = ray.d.x;
  rayhit.ray.dir_y = ray.d.y;
  rayhit.ray.dir_z = ray.d.z;
  rayhit.ray.tnear = ray.tmin;
  rayhit.ray.tfar = ray.tmax;
  rayhit.ray.mask = -1;
  rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
  rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
  rayhit.it = &it;

  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  args.intersect = intersect_func;
  args.filter = filter_func;
  args.feature_mask = RTCFeatureFlags(RTC_FEATURE_FLAG_TRIANGLE | RTC_FEATURE_FLAG_INSTANCE |
                                      RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS |
                                      RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_ARGUMENTS);
  rtcIntersect1(RTCScene(rtc_scene.get()), &rayhit, &args);
  if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID) return false;
  ray.tmax = rayhit.ray.tfar;

  auto instancing_index = uint32_t(-1);
  auto instance_index = uint32_t(-1);
  if (rayhit.hit.instID[0] != RTC_INVALID_GEOMETRY_ID) {
    instancing_index = indices_to_instancing_index[rayhit.hit.instID[0]];
    instance_index = indices_to_instance_index[rayhit.hit.instID[0]];
    it.shape = &scene->instancings[instancing_index].shape;
    it._material = scene->instancings[instancing_index].instances[instance_index].material.get();
  } else {
    it.shape = &scene->geometries[rayhit.hit.geomID]->shape;
    it._material = scene->geometries[rayhit.hit.geomID]->material.get();
  }

  if (it.shape->is<Mesh>()) {
    const auto& mesh = it.shape->as<Mesh>();
    auto face = mesh.indices[rayhit.hit.primID];
    auto uv = vec2(rayhit.hit.u, rayhit.hit.v);

    it.p = mesh.position_of(face, uv);
    if (auto n = mesh.normal_of(face, uv))
      it.n = *n;
    else
      it.n = normalize(vec3(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z));
    if (auto texcoord = mesh.texcoord_of(face, uv))
      it.uv = *texcoord;
    else
      it.uv = uv;
  } else {
    it.shape->compute_surface_info(ray(), it);
  }

  if (instancing_index != uint32_t(-1))
    it.p = vec3(scene->instancings[instancing_index].instances[instance_index].transform *
                vec4(it.p, 1.0f));

  return true;
}

}  // namespace pine