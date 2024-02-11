#pragma once
#include <pine/core/scene.h>
#include <pine/core/array.h>

namespace pine {

struct Voxel {
  float opacity = 0.0f;
  vec3 color;
  int nsamples = 0;
};

Array3d<Voxel> voxelize(const Scene& scene, vec3i resolution);
psl::vector<Array3d<Voxel>> build_mipmap(Array3d<Voxel> original);

}  // namespace pine
