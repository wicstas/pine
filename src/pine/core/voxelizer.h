#pragma once
#include <pine/core/scene.h>
#include <pine/core/array.h>

#include <psl/unordered_map.h>
#include <psl/optional.h>

namespace pine {

struct Voxel {
  vec3 opacity;
  vec3 color;
  int nsamples = 0;
};

template <typename T>
struct SparseArray3d {
  SparseArray3d() = default;
  SparseArray3d(vec3i64 size) : size_(size) {
    storage.reserve(volume(size) / max_value(size));
  }

  T &operator[](vec3i64 p) {
    DCHECK_RANGE(p[0], 0, size_[0] - 1);
    DCHECK_RANGE(p[1], 0, size_[1] - 1);
    DCHECK_RANGE(p[2], 0, size_[2] - 1);
    return storage[p[0] + p[1] * size_[0] + p[2] * size_[0] * size_[1]];
  }

  psl::optional<T> find(vec3i64 p) const {
    DCHECK_RANGE(p[0], 0, size_[0] - 1);
    DCHECK_RANGE(p[1], 0, size_[1] - 1);
    DCHECK_RANGE(p[2], 0, size_[2] - 1);
    auto it = storage.find(p[0] + p[1] * size_[0] + p[2] * size_[0] * size_[1]);
    if (it == storage.end())
      return psl::nullopt;
    else
      return it->second;
  }
  void insert(vec3i64 p, T value) {
    storage.insert(storage.end(),
                   {p[0] + p[1] * size_[0] + p[2] * size_[0] * size_[1], psl::move(value)});
  }

  vec3i64 size() const {
    return size_;
  }

private:
  vec3i64 size_;
  psl::unordered_map<size_t, T> storage;
};

using Voxels = SparseArray3d<Voxel>;

Voxels voxelize(const Scene &scene, AABB aabb, vec3i resolution);
psl::vector<Voxels> build_mipmap(Voxels original);

}  // namespace pine
