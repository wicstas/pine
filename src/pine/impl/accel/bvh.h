#pragma once

#include <pine/core/scene.h>

#include <psl/memory.h>
#include <psl/vector.h>

namespace pine {

class BVHImpl {
private:
  struct alignas(16) Node {
    float surface_area() const;
    AABB get_aabb() const;
    void update_aabb(Node* nodes);
    float compute_cost(Node* nodes);
    float inefficiency() const;

    AABB aabbs[2];

    int children[2] = {-1, -1};
    int parent = -1;
    int index = -1;
    int indexAsChild = -1;
    bool removed = false;

    psl::vector<int> primitiveIndices;
  };

public:
  struct Primitive {
    AABB aabb;
    int index = 0;
  };

  void build(psl::vector<Primitive> primitives);

private:
  int build_sah_binned(Primitive* begin, Primitive* end, AABB aabb);
  void optimize();

public:
  template <typename F>
  bool hit(const Ray& ray, F&& f) const;
  template <typename F>
  bool Intersect(Ray& ray, Interaction& it, F&& f) const;

  AABB get_aabb() const {
    return union_(nodes[rootIndex].aabbs[0], nodes[rootIndex].aabbs[1]);
  }

  int rootIndex = -1;

private:
  psl::vector<Node> nodes;
};

class BVH {
public:
  BVH() = default;
  void build(const Scene* scene);
  bool hit(Ray ray) const;
  bool intersect(Ray& ray, Interaction& it) const;

private:
  psl::vector<BVHImpl> lbvh;
  BVHImpl tbvh;
  psl::vector<int> indices;
  const Scene* scene;
};

}  // namespace pine