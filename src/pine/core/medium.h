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

  void intersect_tr(const Ray& ray, vec3& tmax, SpectralMediumInteraction& mit,
                    Sampler& sampler) const;
  vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const;
  AABB get_aabb() const {
    return aabb;
  }

private:
  psl::shared_ptr<psl::vector<psl::shared_ptr<Geometry>>> geometry;
  AABB aabb;
  Accel accel;
  vec3 sigma_s;
  vec3 sigma_z;
  float max_dim;
};

struct VDBMedium {
  VDBMedium(psl::string filename, mat4 transform, vec3 sigma_s, vec3 sigma_z);
  void intersect_tr(const Ray& ray, vec3& tmax, SpectralMediumInteraction& mit,
                    Sampler& sampler) const;
  vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const;
  AABB get_aabb() const {
    return AABB(bbox);
  }

private:
  psl::opaque_shared_ptr handle;
  void* grid;
  OBB bbox;
  float sigma_maj;
  float sigma_maj_inv;
  vec3 sigma_s;
  vec3 sigma_z;
  vec3 index_start;
  vec3 index_end;
  mat4 world2index;
};

struct Medium : psl::variant<HomogeneousMedium, VDBMedium> {
  void intersect_tr(const Ray& ray, vec3& tmax, SpectralMediumInteraction& mit,
                    Sampler& sampler) const {
    return dispatch([&](auto&& x) { return x.intersect_tr(ray, tmax, mit, sampler); });
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
