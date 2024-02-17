#pragma once
#include <pine/core/sparse_array.h>

#include <psl/vector.h>

namespace pine {

struct Voxel {
  vec3 opacity;
  vec3 color;
  int nsamples = 0;
};

using Voxels = SparseArray3d<Voxel>;

Voxels voxelize(const Scene &scene, AABB aabb, vec3i resolution);
psl::vector<Voxels> build_mipmap(Voxels original);

}  // namespace pine
