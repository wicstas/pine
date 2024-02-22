#include <pine/core/sampling.h>
#include <pine/core/geometry.h>
#include <pine/core/atomic.h>
#include <pine/core/array.h>

#include <psl/optional.h>
#include <psl/vector.h>
#include <psl/array.h>

namespace pine {

struct RadianceSample {
  RadianceSample() = default;
  RadianceSample(vec3 w, vec3 flux) : w{w}, flux{flux} {
  }
  vec3 w;
  vec3 flux;
};

struct PgSample {
  vec3 wo;
  float pdf = 0;
};

struct QuadNode {
  QuadNode(int depth) : depth(depth) {
  }

  float footprint(vec2 sc) const {
    if (is_leaf()) [[unlikely]]
      return length();
    else [[likely]]
      return child(sc).footprint(sc);
  }

  void add_sample(vec2 sc, float flux) {
    this->flux += flux;
    if (!is_leaf()) [[likely]]
      child(sc).add_sample(sc, flux);
  }
  psl::optional<PgSample> sample(vec2 u, vec2 p = vec2(0.0f), float pdf = 1) const {
    if (is_leaf()) [[unlikely]] {
      return PgSample(uniform_sphere(p + u * length()), pdf / (4 * Pi));
    } else [[likely]] {
      if (flux == 0.0f)
        return psl::nullopt;
      auto& c0 = child(0);
      auto& c2 = child(2);
      auto flux_left = c0.flux + c2.flux;
      auto rx = flux_left / flux;
      if (u[0] < rx) {
        u[0] = psl::min(u[0] / rx, one_minus_epsilon);
        auto ry = c0.flux / flux_left;
        if (u[1] < ry) {
          u[1] = psl::min(u[1] / ry, one_minus_epsilon);
          return c0.sample(u, p, pdf * 4 * rx * ry);
        } else {
          p.y += 0.5f * length();
          u[1] = psl::min((u[1] - ry) / (1 - ry), one_minus_epsilon);
          return c2.sample(u, p, pdf * 4 * rx * (1 - ry));
        }
      } else {
        p.x += 0.5f * length();
        auto& c1 = child(1);
        u[0] = psl::min((u[0] - rx) / (1 - rx), one_minus_epsilon);
        auto ry = c1.flux / (flux - flux_left);
        if (u[1] < ry) {
          u[1] = psl::min(u[1] / ry, one_minus_epsilon);
          return c1.sample(u, p, pdf * 4 * (1 - rx) * ry);
        } else {
          p.y += 0.5f * length();
          u[1] = psl::min((u[1] - ry) / (1 - ry), one_minus_epsilon);
          return child(3).sample(u, p, pdf * 4 * (1 - rx) * (1 - ry));
        }
      }
    }
  }
  float pdf(vec2 sc) const {
    if (is_leaf()) [[unlikely]] {
      return 1.0f / (4 * Pi);
    } else [[likely]] {
      if (flux == 0.0f)
        return 0.0f;
      auto& c = child(sc);
      return 4 * c.flux / flux * c.pdf(sc);
    }
  }
  void clear() {
    flux = 0;
    if (!is_leaf())
      for (int i = 0; i < 4; i++)
        child(i).clear();
  }

  void refine(float total_flux) {
    if (flux > total_flux * 0.01f && depth < 15) {
      if (is_leaf()) {
        children = psl::array_of(QuadNode(depth + 1), QuadNode(depth + 1), QuadNode(depth + 1),
                                 QuadNode(depth + 1));
        for (int i = 0; i < 4; i++) {
          child(i).flux = flux / 4;
          child(i).refine(total_flux);
        }
      } else {
        for (int i = 0; i < 4; i++)
          child(i).refine(total_flux);
      }
    } else {
      if (!is_leaf())
        children.reset();
    }
  }
  int node_count() const {
    if (is_leaf())
      return 1;
    else
      return 1 + child(0).node_count() + child(1).node_count() + child(2).node_count() +
             child(3).node_count();
  }
  float length() const {
    return 1.0f / (1 << depth);
  }

private:
  bool is_leaf() const {
    return !children;
  }
  QuadNode& child(vec2& sc) {
    auto index = 0;
    if (sc[0] < 0.5f) {
      sc[0] *= 2;
    } else {
      index += 1;
      sc[0] = (sc[0] - 0.5f) * 2;
    }
    if (sc[1] < 0.5f) {
      sc[1] *= 2;
    } else {
      index += 2;
      sc[1] = (sc[1] - 0.5f) * 2;
    }
    return (*children)[index];
  }
  const QuadNode& child(vec2& sc) const {
    return const_cast<QuadNode*>(this)->child(sc);
  }
  QuadNode& child(int index) {
    return (*children)[index];
  }
  const QuadNode& child(int index) const {
    return (*children)[index];
  }

  friend struct QuadTree;
  Atomic<float> flux{0.0f};
  int depth = -1;  // use uint8_t
  psl::Box<psl::Array<QuadNode, 4>> children;
};

struct QuadTree {
  QuadTree() : root(0) {
  }

  void add_sample(vec3 w, float flux, vec2) {
    auto sc = inverse_uniform_sphere(w);
    // auto footprint = root.footprint(sc);
    // sc += footprint * (u - vec2(0.5f));
    // sc.x = psl::fract(sc.x);
    // sc.y = psl::clamp(sc.y, 0.0f, 1.0f);
    root.add_sample(sc, flux);
    n_samples += 1;
  }
  psl::optional<PgSample> sample(vec2d u) const {
    return root.sample(u);
  }
  float pdf(vec3 w) const {
    return root.pdf(inverse_uniform_sphere(w));
  }
  void clear() {
    root.clear();
  }
  void initial_refinement() {
    root.flux = 1;
    refine();
    clear();
  }
  void refine() {
    root.refine(root.flux);
  }
  int node_count() const {
    return root.node_count();
  }

  QuadNode root;
  Atomic<int> n_samples{0};
};

struct SpatialNode {
  SpatialNode() = default;
  SpatialNode(AABB aabb, QuadTree quad)
      : axis(max_axis(aabb.diagonal())),
        footprint_(aabb.diagonal()[axis] / 2),
        guide(quad),
        collector(psl::move(quad)) {
  }
  psl::optional<PgSample> sample(vec2 u) const {
    return guide->sample(u);
  }
  float pdf(vec3 w) const {
    return guide->pdf(w);
  }
  SpatialNode& traverse(vec3 p) {
    if (is_leaf()) [[unlikely]]
      return *this;
    else [[likely]]
      return child(p).traverse(p);
  }
  const SpatialNode& traverse(vec3 p) const {
    if (is_leaf()) [[unlikely]]
      return *this;
    else [[likely]]
      return child(p).traverse(p);
  }

  void refine(int k, AABB aabb) {
    if (is_leaf()) {
      const auto c = 2000;
      const auto threshold = size_t(c * psl::sqrt<float>(1 << k));
      if (n_samples > threshold) {
        auto [laabb, raabb] = aabb.split_half(axis);
        children =
            psl::array_of(SpatialNode(aabb, *collector), SpatialNode(aabb, psl::move(*collector)));
        guide = collector = psl::nullopt;
        child(0).n_samples = n_samples / 2;
        child(1).n_samples = n_samples / 2;
        child(0).refine(k, laabb);
        child(1).refine(k, raabb);
      } else {
        collector->refine();
        guide = collector;
        collector->clear();
      }
    } else {
      auto [laabb, raabb] = aabb.split_half(axis);
      child(0).refine(k, laabb);
      child(1).refine(k, raabb);
    }
    n_samples = 0;
  }
  void initial_refinement(int64_t n_samples_, AABB aabb) {
    CHECK(is_leaf());
    n_samples = n_samples_;
    collector->initial_refinement();
    refine(0, aabb);
  }

  float footprint() const {
    return footprint_;
  }
  int max_tree_depth() const {
    if (is_leaf())
      return 0;
    else
      return 1 + psl::max(child(0).max_tree_depth(), child(1).max_tree_depth());
  }
  int max_sample_count() const {
    if (is_leaf())
      return guide->n_samples;
    else
      return psl::max(child(0).max_sample_count(), child(1).max_sample_count());
  }
  int node_count() const {
    if (is_leaf())
      return 1;
    else
      return 1 + child(0).node_count() + child(1).node_count();
  }
  int max_quad_node_count() const {
    if (is_leaf())
      return guide->node_count();
    else
      return psl::max(child(0).node_count(), child(1).node_count());
  }

private:
  friend struct SpatialNodeRoot;
  bool is_leaf() const {
    return !children;
  }
  SpatialNode& child(int index) {
    return (*children)[index];
  }
  const SpatialNode& child(int index) const {
    return (*children)[index];
  }
  SpatialNode& child(vec3& p) {
    if (p[axis] < 0.5f) {
      p[axis] = p[axis] * 2;
      return child(0);
    } else {
      p[axis] = (p[axis] - 0.5f) * 2;
      return child(1);
    }
  }
  const SpatialNode& child(vec3& p) const {
    return const_cast<SpatialNode*>(this)->child(p);
  }

  int axis = -1;
  float footprint_ = 0.0f;
  Atomic<size_t> n_samples{0};
  psl::Box<psl::Array<SpatialNode, 2>> children;
  psl::optional<QuadTree> guide;
  psl::optional<QuadTree> collector;
};
struct SpatialNodeRoot {
  SpatialNodeRoot() = default;
  SpatialNodeRoot(AABB aabb, QuadTree quad) : aabb(aabb), root(aabb, quad) {
  }

  void add_sample(SpatialNode& leaf, vec3 p, RadianceSample s, vec3 u, vec2 ud) {
    p += leaf.footprint() * (u - vec3(0.5f));
    p = clamp(p, aabb.lower, aabb.upper);
    auto& chosen_leaf = traverse(p);
    chosen_leaf.n_samples += 1;
    chosen_leaf.collector->add_sample(s.w, length(s.flux), ud);
  }
  SpatialNode& traverse(vec3 p) {
    return root.traverse(aabb.relative_position(p));
  }
  const SpatialNode& traverse(vec3 p) const {
    return root.traverse(aabb.relative_position(p));
  }
  void refine(int k) {
    return root.refine(k, aabb);
  }
  void initial_refinement(int64_t n_samples_) {
    return root.initial_refinement(n_samples_, aabb);
  }
  int max_tree_depth() const {
    return root.max_tree_depth();
  }
  int max_sample_count() const {
    return root.max_sample_count();
  }
  int node_count() const {
    return root.node_count();
  }
  int max_quad_node_count() const {
    return root.max_quad_node_count();
  }

private:
  AABB aabb;
  SpatialNode root;
};

struct SpatialGrid {
  struct Unit {
    QuadTree guide, collector;
  };
  SpatialGrid() = default;
  SpatialGrid(AABB aabb, vec3i resolution) : aabb(aabb), grid(resolution) {
  }

  void add_sample(vec3 p, RadianceSample s, vec3 u, vec2 ud) {
    CHECK_GE(s.flux[0], 0.0f);
    CHECK_GE(s.flux[1], 0.0f);
    CHECK_GE(s.flux[2], 0.0f);
    p += footprint() * (u - vec3(0.5f));
    for (int i = 0; i < 3; i++) {
      if (p[i] < aabb.lower[i])
        p[i] += 2 * (aabb.lower[i] - p[i]);
      else if (p[i] > aabb.upper[i])
        p[i] += 2 * (aabb.upper[i] - p[i]);
    }
    auto& leaf = traverse(p);
    leaf.collector.add_sample(s.w, length(s.flux), ud);
  }

  void refine(int) {
    for (auto& unit : grid) {
      unit.collector.refine();
      unit.guide = unit.collector;
      unit.collector.clear();
    }
  }
  void initial_refinement(int64_t) {
  }

  psl::optional<PgSample> sample(vec3 p, vec2 u) const {
    return traverse(p).guide.sample(u);
  }
  float pdf(vec3 p, vec3 w) const {
    return traverse(p).guide.pdf(w);
  }

private:
  float footprint() const {
    return max_value(aabb.diagonal() / grid.size());
  }
  Unit& traverse(vec3 p) {
    return grid[clamp(vec3i(aabb.relative_position(p) * grid.size()), vec3i(0),
                      vec3i(grid.size()) - vec3i(1))];
  }
  const Unit& traverse(vec3 p) const {
    return grid[clamp(vec3i(aabb.relative_position(p) * grid.size()), vec3i(0),
                      vec3i(grid.size()) - vec3i(1))];
  }

  AABB aabb;
  Array3d<Unit> grid;
};

using SpatialTree = SpatialNodeRoot;

}  // namespace pine