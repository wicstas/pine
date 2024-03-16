#pragma once
#include <pine/core/interaction.h>
#include <pine/core/vecmath.h>
#include <pine/core/bbox.h>

#include <psl/function.h>
#include <psl/variant.h>

namespace pine {

struct HomogeneousMedium {
  HomogeneousMedium(AABB aabb, float sigma_a, float sigma_s)
      : aabb(aabb), sigma_s(sigma_s), sigma_z(sigma_a + sigma_s) {
  }

  psl::optional<MediumInteraction> intersect_tr(Ray& ray, Sampler& sampler) const;
  vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const;

private:
  AABB aabb;
  float sigma_s;
  float sigma_z;
};

struct VDBMedium {
  VDBMedium(psl::string filename, mat4 transform, float sigma_s, float sigma_z);
  psl::optional<MediumInteraction> intersect_tr(Ray& ray, Sampler& sampler) const;
  vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const;

private:
  psl::opaque_shared_ptr handle;
  void* grid;
  OBB bbox;
  float sigma_maj;
  float sigma_maj_inv;
  float sigma_s;
  float sigma_z;
  vec3 index_start;
  vec3 index_end;
  mat4 world2index;
};

struct Medium : psl::variant<HomogeneousMedium, VDBMedium> {
  psl::optional<MediumInteraction> intersect_tr(Ray& ray, Sampler& sampler) const {
    auto ms = dispatch([&](auto&& x) { return x.intersect_tr(ray, sampler); });
    return ms;
  }
  vec3 transmittance(vec3 p, vec3 d, float tmax, Sampler& sampler) const {
    return dispatch([&](auto&& x) { return x.transmittance(p, d, tmax, sampler); });
  }
};

void medium_context(Context& context);

}  // namespace pine
