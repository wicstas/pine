#pragma once

#include <pine/core/scene.h>

#include <psl/memory.h>
#include <psl/vector.h>

namespace pine {

class BVHImpl {
public:
  struct alignas(16) Node {
    float surface_area() const {
      return union_(aabbs[0], aabbs[1]).surface_area();
    }
    AABB get_aabb() const {
      return union_(aabbs[0], aabbs[1]);
    }
    void UpdateAABB(Node* nodes) {
      aabbs[0] = nodes[children[0]].get_aabb();
      aabbs[1] = nodes[children[1]].get_aabb();
      if (parent != -1)
        nodes[parent].UpdateAABB(nodes);
    }
    float ComputeCost(Node* nodes) {
      if (primitiveIndices.size())
        return surface_area();
      return surface_area() + nodes[children[0]].ComputeCost(nodes) +
             nodes[children[1]].ComputeCost(nodes);
    }
    float Inefficiency() const {
      float mSum = surface_area() / (2 * (aabbs[0].surface_area() + aabbs[1].surface_area()));
      float mMin = surface_area() / psl::min(aabbs[0].surface_area(), aabbs[1].surface_area());
      float mArea = surface_area();
      return mSum * mMin * mArea;
    }

    AABB aabbs[2];

    int children[2] = {-1, -1};
    int parent = -1;
    int index = -1;
    int indexAsChild = -1;
    bool removed = false;

    psl::vector<int> primitiveIndices;
  };
  struct Primitive {
    AABB aabb;
    int index = 0;
  };

  void Build(psl::vector<Primitive> primitives);

  int BuildSAHBinned(Primitive* begin, Primitive* end, AABB aabb);
  void Optimize();

  template <typename F>
  bool hit(const Ray& ray, F&& f) const;
  template <typename F>
  bool Intersect(Ray& ray, Interaction& it, F&& f) const;
  AABB get_aabb() const {
    return union_(nodes[rootIndex].aabbs[0], nodes[rootIndex].aabbs[1]);
  }

  int rootIndex = -1;
  psl::vector<Node> nodes;
};

class BVH {
public:
  BVH() = default;
  void build(const Scene* scene);
  bool hit(Ray ray) const;
  bool intersect(Ray& ray, Interaction& it) const;

  psl::vector<BVHImpl> lbvh;
  BVHImpl tbvh;
  psl::vector<int> indices;
  const Scene* scene;
};

}  // namespace pine