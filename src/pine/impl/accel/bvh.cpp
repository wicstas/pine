#include <pine/impl/accel/bvh.h>
#include <pine/core/profiler.h>
#include <pine/core/scene.h>
#include <pine/core/rng.h>

#include <queue>

namespace pine {

void BVHImpl::Build(psl::vector<Primitive> primitives) {
  Profiler _("Build BVH");
  Timer timer;

  AABB aabb;
  for (auto& primitive : primitives)
    aabb.extend(primitive.aabb);

  nodes.reserve(primitives.size());
  BuildSAHBinned(primitives.data(), primitives.data() + primitives.size(), aabb);
  rootIndex = (int)nodes.size() - 1;
  // Optimize();
}

int BVHImpl::BuildSAHBinned(Primitive* begin, Primitive* end, AABB aabb) {
  Node node;
  int numPrimitives = int(end - begin);
  CHECK_NE(numPrimitives, 0);

  auto MakeLeaf = [&]() {
    for (int i = 0; i < numPrimitives; i++)
      node.primitiveIndices.push_back(begin[i].index);
    for (Primitive* prim = begin; prim != end; prim++)
      node.aabbs[0].extend(prim->aabb);
    node.aabbs[1] = node.aabbs[0];
    node.index = (int)nodes.size();
    nodes.push_back(node);
    return node.index;
  };
  if (numPrimitives == 1)
    return MakeLeaf();

  AABB aabbCentroid;
  for (int i = 0; i < numPrimitives; i++)
    aabbCentroid.extend(begin[i].aabb.Centroid());
  float surfaceArea = aabb.SurfaceArea();

  struct Bucket {
    int count = 0;
    AABB aabb;
  };
  const int nBuckets = 16;

  float minCost = FloatMax;
  int bestAxis = -1;
  int splitBucket = -1;

  for (int axis = 0; axis < 3; axis++) {
    if (!aabbCentroid.is_valid(axis))
      continue;

    Bucket buckets[nBuckets];

    for (int i = 0; i < numPrimitives; i++) {
      int b = psl::min(int(nBuckets * aabbCentroid.Offset(begin[i].aabb.Centroid(axis), axis)),
                       nBuckets - 1);
      buckets[b].count++;
      buckets[b].aabb.extend(begin[i].aabb);
    }

    float cost[nBuckets - 1] = {};

    AABB bForward;
    int countForward = 0;
    for (int i = 0; i < nBuckets - 1; i++) {
      bForward.extend(buckets[i].aabb);
      countForward += buckets[i].count;
      cost[i] += countForward * bForward.SurfaceArea();
    }

    AABB bBackward;
    int countBackward = 0;
    for (int i = nBuckets - 1; i >= 1; i--) {
      bBackward.extend(buckets[i].aabb);
      countBackward += buckets[i].count;
      cost[i - 1] += countBackward * bBackward.SurfaceArea();
    }

    for (int i = 0; i < nBuckets - 1; i++) {
      cost[i] = 1.0f + cost[i] / surfaceArea;
    }

    float axisMinCost = FloatMax;
    int axisSplitBucket = -1;
    for (int i = 0; i < nBuckets - 1; i++) {
      if (cost[i] < axisMinCost) {
        axisMinCost = cost[i];
        axisSplitBucket = i;
      }
    }

    if (axisMinCost < minCost) {
      minCost = axisMinCost;
      bestAxis = axis;
      splitBucket = axisSplitBucket;
    }
  }

  float leafCost = numPrimitives;
  if (minCost > leafCost)
    return MakeLeaf();

  Primitive* pmid = psl::partition(psl::range(begin, end), [=](const Primitive& prim) {
    int b = nBuckets * aabbCentroid.Offset(prim.aabb.Centroid(bestAxis), bestAxis);
    if (b == nBuckets)
      b = nBuckets - 1;
    return b <= splitBucket;
  });

  CHECK(begin != pmid);
  CHECK(end != pmid);

  for (Primitive* prim = begin; prim != pmid; prim++)
    node.aabbs[0].extend(prim->aabb);
  for (Primitive* prim = pmid; prim != end; prim++)
    node.aabbs[1].extend(prim->aabb);
  node.children[0] = BuildSAHBinned(begin, pmid, node.aabbs[0]);
  node.children[1] = BuildSAHBinned(pmid, end, node.aabbs[1]);
  node.index = (int)nodes.size();
  nodes[node.children[0]].parent = node.index;
  nodes[node.children[1]].parent = node.index;
  nodes[node.children[0]].indexAsChild = 0;
  nodes[node.children[1]].indexAsChild = 1;

  nodes.push_back(node);
  return node.index;
}

struct Item {
  Item(int nodeIndex, float costInduced, float inverseCost)
      : nodeIndex(nodeIndex), costInduced(costInduced), inverseCost(inverseCost){};

  friend bool operator<(Item l, Item r) {
    return l.inverseCost < r.inverseCost;
  }

  int nodeIndex;
  float costInduced;
  float inverseCost;
};
struct pair {
  friend bool operator<(pair l, pair r) {
    return l.inefficiency > r.inefficiency;
  }

  int index = -1;
  float inefficiency;
};
void BVHImpl::Optimize() {
  auto FindNodeForReinsertion = [&](int nodeIndex) {
    float eps = 1e-20f;
    float costBest = FloatMax;
    int nodeBest = -1;

    auto& L = nodes[nodeIndex];

    std::priority_queue<Item> queue;
    queue.push(Item(rootIndex, 0.0f, FloatMax));

    while (queue.size()) {
      Item item = queue.top();
      queue.pop();
      auto CiLX = item.costInduced;
      auto& X = nodes[item.nodeIndex];

      if (CiLX + L.SurfaceArea() >= costBest)
        break;

      float CdLX = Union(X.get_aabb(), L.get_aabb()).SurfaceArea();
      float CLX = CiLX + CdLX;
      if (CLX < costBest) {
        costBest = CLX;
        nodeBest = item.nodeIndex;
      }

      float Ci = CLX - X.SurfaceArea();

      if (Ci + L.SurfaceArea() < costBest) {
        if (X.primitiveIndices.size() == 0) {
          queue.push(Item(X.children[0], Ci, 1.0f / (Ci + eps)));
          queue.push(Item(X.children[1], Ci, 1.0f / (Ci + eps)));
        }
      }
    }

    return nodeBest;
  };

  RNG sampler;
  float startCost = nodes[rootIndex].ComputeCost(&nodes[0]) / 100000.0f;
  float lastCost = startCost;
  int numConvergedPasses = 0;
  for (int pass = 0; pass < 256; pass++) {
    if (pass % 5 == 1) {
      float cost = nodes[rootIndex].ComputeCost(&nodes[0]) / 100000.0f;
      if (cost < lastCost * 0.99f) {
        numConvergedPasses = 0;
        lastCost = cost;
      } else {
        numConvergedPasses++;
        if (numConvergedPasses > 1) {
          break;
        }
      }
    }

    psl::vector<psl::pair<int, int>> unusedNodes;
    if (pass % 3 == 0) {
      psl::vector<pair> inefficiencies(nodes.size());

      for (int i = 0; i < (int)nodes.size(); i++)
        inefficiencies[i] = {i, nodes[i].Inefficiency()};
      psl::partial_sort(inefficiencies, inefficiencies.begin() + nodes.size() / 200,
                        psl::less<pair>{});

      for (int i = 0; i < (int)nodes.size() / 200; i++) {
        int nodeIndex = inefficiencies[i].index;
        if (nodes[nodeIndex].primitiveIndices.size() != 0 || nodes[nodeIndex].removed ||
            nodes[nodeIndex].parent == -1 || nodes[nodes[nodeIndex].parent].removed ||
            nodes[nodes[nodeIndex].parent].parent == -1 ||
            nodes[nodes[nodes[nodeIndex].parent].parent].removed ||
            nodes[nodes[nodes[nodeIndex].parent].parent].parent == -1)
          continue;

        Node& node = nodes[nodeIndex];
        Node& parent = nodes[node.parent];
        Node& grandparent = nodes[parent.parent];
        node.removed = true;
        parent.removed = true;

        int secondChildOfParent = parent.children[1 - node.indexAsChild];
        grandparent.children[parent.indexAsChild] = secondChildOfParent;
        nodes[secondChildOfParent].indexAsChild = parent.indexAsChild;
        nodes[secondChildOfParent].parent = grandparent.index;
        grandparent.UpdateAABB(&nodes[0]);

        nodes[node.children[0]].removed = true;
        nodes[node.children[1]].removed = true;

        unusedNodes.push_back({node.children[0], nodeIndex});
        unusedNodes.push_back({node.children[1], node.parent});
      }
    } else {
      for (int i = 0; i < (int)nodes.size() / 100; i++) {
        int nodeIndex = -1;
        do {
          nodeIndex = sampler.Uniform64u() % nodes.size();
        } while ((nodes[nodeIndex].primitiveIndices.size() != 0 || nodes[nodeIndex].removed ||
                  nodes[nodeIndex].parent == -1 || nodes[nodes[nodeIndex].parent].removed ||
                  nodes[nodes[nodeIndex].parent].parent == -1 ||
                  nodes[nodes[nodes[nodeIndex].parent].parent].removed ||
                  nodes[nodes[nodes[nodeIndex].parent].parent].parent == -1));

        Node& node = nodes[nodeIndex];
        Node& parent = nodes[node.parent];
        Node& grandparent = nodes[parent.parent];
        node.removed = true;
        parent.removed = true;

        int secondChildOfParent = parent.children[1 - node.indexAsChild];
        grandparent.children[parent.indexAsChild] = secondChildOfParent;
        nodes[secondChildOfParent].indexAsChild = parent.indexAsChild;
        nodes[secondChildOfParent].parent = grandparent.index;
        grandparent.UpdateAABB(&nodes[0]);

        nodes[node.children[0]].removed = true;
        nodes[node.children[1]].removed = true;

        unusedNodes.push_back({node.children[0], nodeIndex});
        unusedNodes.push_back({node.children[1], node.parent});
      }
    }

    psl::sort(unusedNodes, [&](psl::pair<int, int> l, psl::pair<int, int> r) {
      return nodes[l.first].SurfaceArea() > nodes[r.first].SurfaceArea();
    });

    for (const auto& node : unusedNodes) {
      int L = node.first;
      int N = node.second;
      Node& x = nodes[FindNodeForReinsertion(L)];
      Node& n = nodes[N];
      Node& l = nodes[L];

      if (x.parent != -1)
        nodes[x.parent].children[x.indexAsChild] = n.index;
      else
        rootIndex = n.index;
      n.parent = x.parent;
      n.indexAsChild = x.indexAsChild;
      n.removed = false;

      n.children[0] = x.index;
      n.children[1] = l.index;
      x.parent = n.index;
      l.parent = n.index;

      x.indexAsChild = 0;
      l.indexAsChild = 1;
      l.removed = false;
      n.UpdateAABB(&nodes[0]);
    }
  }
}

template <typename F>
bool BVHImpl::hit(const Ray& ray, F&& f) const {
  RayOctant rayOctant = RayOctant(ray);
  const Node* PINE_RESTRICT nodes = this->nodes.data();

  constexpr int max_stack_size = 32;
  int stack[max_stack_size];
  int ptr = 0;
  int next = rootIndex;

  if (PINE_UNLIKELY(nodes[next].primitiveIndices.size())) {
    for (int index : nodes[next].primitiveIndices)
      if (f(ray, index))
        return true;
  } else {
    while (true) {
      const Node& node = nodes[next];

      int leftChildIndex = -1, rightChildIndex = -1;
      float t0 = ray.tmax, t1 = ray.tmax;
      if (node.aabbs[0].hit(rayOctant, ray.tmin, &t0)) {
        const Node& leftChild = nodes[node.children[0]];
        if (PINE_LIKELY(!leftChild.primitiveIndices.size())) {
          leftChildIndex = node.children[0];
        } else {
          for (int index : leftChild.primitiveIndices)
            if (f(ray, index))
              return true;
        }
      }
      if (node.aabbs[1].hit(rayOctant, ray.tmin, &t1)) {
        const Node& rightChild = nodes[node.children[1]];
        if (PINE_LIKELY(!rightChild.primitiveIndices.size())) {
          rightChildIndex = node.children[1];

        } else {
          for (int index : rightChild.primitiveIndices)
            if (f(ray, index))
              return true;
        }
      }

      if (leftChildIndex != -1) {
        if (rightChildIndex != -1) {
          if (t0 > t1) {
            DCHECK_LT(ptr, max_stack_size);
            stack[ptr++] = leftChildIndex;
            next = rightChildIndex;
          } else {
            DCHECK_LT(ptr, max_stack_size);
            stack[ptr++] = rightChildIndex;
            next = leftChildIndex;
          }
        } else {
          next = leftChildIndex;
        }
      } else if (rightChildIndex != -1) {
        next = rightChildIndex;
      } else {
        if (PINE_UNLIKELY(ptr == 0))
          break;
        next = stack[--ptr];
      }
    }
  }

  return false;
}

template <typename F>
bool BVHImpl::Intersect(Ray& ray, Interaction& it, F&& f) const {
  RayOctant rayOctant = RayOctant(ray);
  const Node* PINE_RESTRICT nodes = this->nodes.data();

  bool hit = false;
  constexpr int max_stack_size = 32;
  int stack[max_stack_size];
  int ptr = 0;
  int next = rootIndex;

  if (PINE_UNLIKELY(nodes[next].primitiveIndices.size())) {
    for (int index : nodes[next].primitiveIndices)
      if (f(ray, it, index))
        hit = true;
  } else {
    while (true) {
      it.bvh += 1;
      const Node& node = nodes[next];

      int leftChildIndex = -1, rightChildIndex = -1;
      float t0 = ray.tmax, t1 = ray.tmax;
      if (node.aabbs[0].hit(rayOctant, ray.tmin, &t0)) {
        const Node& leftChild = nodes[node.children[0]];
        if (PINE_LIKELY(!leftChild.primitiveIndices.size())) {
          leftChildIndex = node.children[0];
        } else {
          for (int index : leftChild.primitiveIndices)
            if (f(ray, it, index))
              hit = true;
        }
      }
      if (node.aabbs[1].hit(rayOctant, ray.tmin, &t1)) {
        const Node& rightChild = nodes[node.children[1]];
        if (PINE_LIKELY(!rightChild.primitiveIndices.size())) {
          rightChildIndex = node.children[1];

        } else {
          for (int index : rightChild.primitiveIndices)
            if (f(ray, it, index))
              hit = true;
        }
      }

      if (leftChildIndex != -1) {
        if (rightChildIndex != -1) {
          if (t0 > t1) {
            DCHECK_LT(ptr, max_stack_size);
            stack[ptr++] = leftChildIndex;
            next = rightChildIndex;
          } else {
            DCHECK_LT(ptr, max_stack_size);
            stack[ptr++] = rightChildIndex;
            next = leftChildIndex;
          }
        } else {
          next = leftChildIndex;
        }
      } else if (rightChildIndex != -1) {
        next = rightChildIndex;
      } else {
        if (PINE_UNLIKELY(ptr == 0))
          break;
        next = stack[--ptr];
      }
    }
  }

  return hit;
}

void BVH::build(const Scene* scene_) {
  scene = scene_;
  if (scene->geometries.size() == 0)
    return;

  for (size_t i = 0; i < scene->geometries.size(); i++) {
    if (scene->geometries[i]->shape.is<TriangleMesh>()) {
      auto& mesh = scene->geometries[i]->shape.be<TriangleMesh>();
      auto primitives = psl::vector<BVHImpl::Primitive>{};
      for (size_t it = 0; it < mesh.num_triangles(); it++) {
        auto primitive = BVHImpl::Primitive{};
        primitive.aabb = mesh.get_aabb(it);
        primitive.index = static_cast<int>(primitives.size());
        primitives.push_back(primitive);
      }
      auto bvh = BVHImpl{};
      bvh.Build(psl::move(primitives));
      lbvh.push_back(psl::move(bvh));
      indices.push_back(static_cast<int>(i));
    }
  }

  psl::vector<BVHImpl::Primitive> primitives;
  for (auto& s : lbvh) {
    auto primitive = BVHImpl::Primitive{};
    primitive.aabb = s.get_aabb();
    primitive.index = static_cast<int>(primitives.size());
    primitives.push_back(primitive);
  }
  for (size_t i = 0; i < scene->geometries.size(); i++) {
    if (!scene->geometries[i]->shape.is<TriangleMesh>()) {
      auto primitive = BVHImpl::Primitive{};
      primitive.aabb = scene->geometries[i]->get_aabb();
      primitive.index = static_cast<int>(primitives.size());
      primitives.push_back(primitive);
      indices.push_back(static_cast<int>(i));
    }
  }
  tbvh.Build(primitives);
}

bool BVH::hit(Ray ray) const {
  if (scene->geometries.size() == 0)
    return false;

  return tbvh.hit(ray, [&](const Ray& ray, int lbvhIndex) {
    auto& geometry = scene->geometries[indices[lbvhIndex]];

    if (lbvhIndex < static_cast<int>(lbvh.size())) {
      return lbvh[lbvhIndex].hit(ray, [&](const Ray& ray, int index) {
        return geometry->shape.be<TriangleMesh>().hit(ray, index);
      });
    } else {
      return geometry->hit(ray);
    }
  });
}

bool BVH::intersect(Ray& ray, Interaction& it) const {
  if (scene->geometries.size() == 0)
    return false;

  return tbvh.Intersect(ray, it, [&](Ray& ray, Interaction& it, int i_lbvh) {
    auto& geometry = scene->geometries[indices[i_lbvh]];

    if (i_lbvh < static_cast<int>(lbvh.size())) {
      auto& mesh = geometry->shape.be<TriangleMesh>();
      auto hit = lbvh[i_lbvh].Intersect(ray, it, [&](Ray& ray, Interaction& it, int index) {
        return mesh.intersect(ray, it, index);
      });
      if (hit) {
        it.geometry = geometry.get();
        it.material = geometry->material.get();
      }
      return hit;
    } else {
      return geometry->intersect(ray, it);
    }
  });
}

}  // namespace pine