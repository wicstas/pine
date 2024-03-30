#pragma once
#include <pine/core/interaction.h>
#include <pine/core/bbox.h>
#include <pine/core/ray.h>

#include <psl/memory.h>
#include <psl/vector.h>
#include <psl/span.h>

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
  bool Intersect(Ray& ray, F&& f) const;

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
  uint8_t hit8(psl::span<const Ray> rays) const {
    auto result = uint8_t(0);
    for (int i = 0; i < 8; i++)
      if (hit(rays[i]))
        result |= 1 << i;
    return result;
  }
  bool intersect(Ray& ray, SurfaceInteraction& it) const;

private:
  psl::vector<BVHImpl> lbvh;
  BVHImpl tbvh;
  psl::vector<int> indices;
  const Scene* scene;
};

}  // namespace pine