#pragma once
#include <pine/core/interaction.h>
#include <pine/core/vecmath.h>
#include <pine/core/accel.h>
#include <pine/core/bbox.h>

#include <psl/function.h>
#include <psl/variant.h>

namespace pine {

struct HomogeneousMedium {
  HomogeneousMedium(Shape shape, vec3 sigma_a, vec3 sigma_s);
  ~HomogeneousMedium();  // = default

  psl::optional<MediumInteraction> intersect_tr(const Ray& ray, Sampler& sampler) const;
  vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const;
  AABB get_aabb() const {
    return aabb;
  }

private:
  psl::shared_ptr<Scene> scene;
  AABB aabb;
  Accel accel;
  vec3 sigma_s;
  vec3 sigma_z;
  float max_dim;
};

struct MajorantGrid {
  struct Node {
    float majorant;
  };

  struct Traverer {
    Ray ray;
    MajorantGrid& grid;
  };

  Traverer traver(const Ray& ray) {
    return Traverer{ray, *this};
  }

private:
  Array3d<Node> grid;
};

struct VDBMedium {
  VDBMedium(psl::string filename, mat4 transform, vec3 sigma_a, vec3 sigma_s,
            float blackbody_intensity = 1.0f, float temperature_scale = 1.0f);
  psl::optional<MediumInteraction> intersect_tr(const Ray& ray, Sampler& sampler) const;
  vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const;
  AABB get_aabb() const {
    return AABB(bbox);
  }

private:
  psl::opaque_shared_ptr density_handle;
  psl::opaque_shared_ptr flame_handle;
  psl::opaque_shared_ptr temperature_handle;
  void* density_grid;
  void* flame_grid;
  void* temperature_grid;
  OBB bbox;
  vec3 sigma_majs;
  vec3 sigma_maj_invs;
  float sigma_maj;
  float sigma_maj_inv;
  vec3 sigma_a;
  vec3 sigma_s;
  vec3 sigma_z;
  mat4 world2index;
  mat4 world2index_flame;
  mat4 world2index_tem;
  float blackbody_intensity = 1.0f;
  float temperature_scale = 1.0f;
};

struct Medium : psl::variant<HomogeneousMedium, VDBMedium> {
  psl::optional<MediumInteraction> intersect_tr(const Ray& ray, Sampler& sampler) const {
    return dispatch([&](auto&& x) { return x.intersect_tr(ray, sampler); });
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
