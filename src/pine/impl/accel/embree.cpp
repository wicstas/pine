#include <pine/impl/accel/embree.h>
#include <pine/core/profiler.h>
#include <pine/core/scene.h>

#include <embree4/rtcore.h>

namespace pine {

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
  if (!args->valid[0])
    return;

  const Shape& shape = *reinterpret_cast<const Shape*>(args->geometryUserPtr);
  auto& ray_ = reinterpret_cast<RTCRayHit*>(args->rayhit)->ray;
  auto& hit_ = reinterpret_cast<RTCRayHit*>(args->rayhit)->hit;

  auto ray = Ray(vec3(ray_.org_x, ray_.org_y, ray_.org_z), vec3(ray_.dir_x, ray_.dir_y, ray_.dir_z),
                 ray_.tnear, ray_.tfar);
  if (shape.intersect(ray)) {
    ray_.tfar = ray.tmax;
    hit_.geomID = args->geomID;
    hit_.primID = args->primID;
    args->valid[0] = -1;
  }
}

static void hit_func(const RTCOccludedFunctionNArguments* args) {
  if (!args->valid[0])
    return;

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

#pragma unroll
  for (int i = 0; i < 8; i++) {
    if (args->valid[i]) {
      if (shape.hit(Ray(vec3(ray_.org_x[i], ray_.org_y[i], ray_.org_z[i]),
                        vec3(ray_.dir_x[i], ray_.dir_y[i], ray_.dir_z[i]), ray_.tnear[i],
                        ray_.tfar[i])))
        ray_.tfar[i] = -Infinity;
    }
  }
}
void EmbreeAccel::build(const psl::vector<psl::shared_ptr<pine::Geometry>>* geometries) {
  this->geometries = geometries;
  if (geometries->size() == 0)
    return;
  rtc_device = psl::opaque_shared_ptr(
      rtcNewDevice(nullptr), +[](RTCDevice ptr) { rtcReleaseDevice(ptr); });
  auto rtc_device = RTCDevice(this->rtc_device.get());
  rtc_scene = psl::opaque_shared_ptr(
      rtcNewScene(rtc_device), +[](RTCScene ptr) { rtcReleaseScene(ptr); });
  auto rtc_scene = RTCScene(this->rtc_scene.get());
  rtcSetSceneBuildQuality(rtc_scene, RTC_BUILD_QUALITY_HIGH);

  for (uint32_t i = 0; i < geometries->size(); i++) {
    if ((*geometries)[i]->shape.is<TriangleMesh>()) {
      auto mesh = (*geometries)[i]->shape.as<TriangleMesh>();
      RTCGeometry geom = rtcNewGeometry(rtc_device, RTC_GEOMETRY_TYPE_TRIANGLE);

      float* vb = (float*)rtcSetNewGeometryBuffer(
          geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(vec3), mesh.vertices.size());
      psl::memcpy(vb, mesh.vertices.data(), mesh.vertices.byte_size());

      unsigned* ib = (unsigned*)rtcSetNewGeometryBuffer(
          geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(vec3u32), mesh.num_triangles());
      psl::memcpy(ib, mesh.indices.data(), mesh.indices.byte_size());
      rtcCommitGeometry(geom);
      CHECK_EQ(i, rtcAttachGeometry(rtc_scene, geom));
      rtcReleaseGeometry(geom);
    } else {
      RTCGeometry geom = rtcNewGeometry(rtc_device, RTC_GEOMETRY_TYPE_USER);
      rtcSetGeometryUserPrimitiveCount(geom, 1);
      rtcSetGeometryUserData(geom, &(*geometries)[i]->shape);
      rtcSetGeometryBoundsFunction(geom, bounds_func, nullptr);
      rtcCommitGeometry(geom);
      CHECK_EQ(i, rtcAttachGeometry(rtc_scene, geom));
      rtcReleaseGeometry(geom);
    }
  }
  rtcCommitScene(rtc_scene);
}
bool EmbreeAccel::hit(Ray ray) const {
  if (geometries->size() == 0)
    return false;
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
  args.feature_mask = RTCFeatureFlags(RTC_FEATURE_FLAG_TRIANGLE |
                                      RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_ARGUMENTS);
  rtcOccluded1(RTCScene(rtc_scene.get()), &ray_, &args);
  return ray_.tfar < 0;
}
uint8_t EmbreeAccel::hit8(psl::span<const Ray> rays) const {
  if (geometries->size() == 0)
    return false;
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
  args.feature_mask = RTCFeatureFlags(RTC_FEATURE_FLAG_TRIANGLE |
                                      RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_ARGUMENTS);
  int valid[8]{-1, -1, -1, -1, -1, -1, -1, -1};
  rtcOccluded8(valid, RTCScene(rtc_scene.get()), &ray_, &args);
  auto result = uint8_t(0);
  for (int i = 0; i < 8; i++)
    if (ray_.tfar[i] < 0)
      result |= 1 << i;

  return result;
}
bool EmbreeAccel::intersect(Ray& ray, SurfaceInteraction& it) const {
  if (geometries->size() == 0)
    return false;
  RTCRayHit rayhit;
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

  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  args.intersect = intersect_func;
  args.feature_mask = RTCFeatureFlags(RTC_FEATURE_FLAG_TRIANGLE |
                                      RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_ARGUMENTS);
  rtcIntersect1(RTCScene(rtc_scene.get()), &rayhit, &args);
  if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID)
    return false;

  ray.tmax = rayhit.ray.tfar;
  it.geometry = (*geometries)[rayhit.hit.geomID].get();

  if (it.geometry->shape.is<TriangleMesh>()) {
    const auto& mesh = it.geometry->shape.as<TriangleMesh>();
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
    it.p = ray();
    (*geometries)[rayhit.hit.geomID]->compute_surface_info(it);
  }

  return true;
}

}  // namespace pine