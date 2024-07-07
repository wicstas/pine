#pragma once
#include <pine/core/interaction.h>
#include <pine/core/vecmath.h>
#include <pine/core/accel.h>
#include <pine/core/bbox.h>

#include <psl/function.h>
#include <psl/variant.h>

namespace pine {

struct MediumSample {
  MediumSample() = default;
  MediumSample(vec3 p, vec3 W, float pdf, PhaseFunction pg) : p(p), W(W), pdf(pdf), pg(MOVE(pg)) {
  }
  Ray spawn_ray(vec3 wo, float tmax = float_max) const {
    return Ray(p, wo, 0.0f, tmax * 0.999f);
  }

  vec3 p;
  vec3 W;
  float pdf;
  PhaseFunction pg;
};

struct MediumPoint {
  vec3 sigma_s;
};

struct HomogeneousMedium {
  HomogeneousMedium(Shape shape, PhaseFunction pf, vec3 sigma_a, vec3 sigma_s);
  ~HomogeneousMedium();  // = default

  psl::optional<MediumSample> sample(const Ray& ray, Sampler& sampler) const;
  MediumPoint at(vec3 p) const;
  const PhaseFunction& get_pf() const {
    return pf;
  }
  vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const;
  AABB get_aabb() const {
    return aabb;
  }

private:
  psl::shared_ptr<Scene> scene;
  AABB aabb;
  Accel accel;
  PhaseFunction pf;
  vec3 sigma_s;
  vec3 sigma_z;
  float max_dim;
};

struct VDBMedium {
  VDBMedium(psl::string filename, mat4 transform, PhaseFunction pf, vec3 sigma_a, vec3 sigma_s,
            float blackbody_intensity = 1.0f, float temperature_scale = 1.0f);
  psl::optional<MediumSample> sample(const Ray& ray, Sampler& sampler) const;
  MediumPoint at(vec3 p) const;
  const PhaseFunction& get_pf() const {
    return pf;
  }
  vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const;
  AABB get_aabb() const {
    return AABB(bbox);
  }

private:
  psl::opaque_shared_ptr density_handle;
  psl::opaque_shared_ptr flame_handle;
  psl::opaque_shared_ptr temperature_handle;
  void* density_grid = nullptr;
  void* flame_grid = nullptr;
  void* temperature_grid = nullptr;
  OBB bbox;
  PhaseFunction pf;
  float sigma_maj;
  float sigma_maj_inv;
  float sigma_a_;
  float sigma_s_;
  float sigma_z_;
  vec3 sigma_a;
  vec3 sigma_s;
  vec3 sigma_z;
  mat4 world2index;
  mat4 world2index_flame;
  mat4 world2index_tem;
  float blackbody_intensity = 1.0f;
  float temperature_scale = 1.0f;
};

// struct LambdaMedium {
//   LambdaMedium(mat4 transform, psl::function<float(vec3)> density, PhaseFunction pf, vec3
//   sigma_a,
//                vec3 sigma_s);
//   psl::optional<MediumSample> sample(const Ray& ray, Sampler& sampler) const;
//   vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const;
//   AABB get_aabb() const {
//     return AABB(bbox);
//   }

// private:
//   psl::function<float(vec3)> density;
//   OBB bbox;
//   PhaseFunction pf;
//   float sigma_maj;
//   float sigma_maj_inv;
//   float sigma_a_;
//   float sigma_s_;
//   float sigma_z_;
//   vec3 sigma_a;
//   vec3 sigma_s;
//   vec3 sigma_z;
// };

struct Medium : psl::variant<HomogeneousMedium, VDBMedium> {
  psl::optional<MediumSample> sample(const Ray& ray, Sampler& sampler) const {
    return dispatch([&](auto&& x) { return x.sample(ray, sampler); });
  }
  MediumPoint at(vec3 p) const {
    return dispatch([&](auto&& x) { return x.at(p); });
  }
  const PhaseFunction& get_pf() const {
    return dispatch([&](auto&& x) -> decltype(auto) { return x.get_pf(); });
  }
  vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const {
    return dispatch([&](auto&& x) { return x.transmittance(p, d, tmax, sampler); });
  }
  AABB get_aabb() const {
    return dispatch([&](auto&& x) { return x.get_aabb(); });
  }
};

void medium_context(Context& context);

}  // namespace pine
