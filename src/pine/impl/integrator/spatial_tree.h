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
  QuadNode() = default;
  QuadNode(vec2 lower, vec2 upper) : lower(lower), upper(upper) {
  }

  float footprint(vec2 sc) const {
    if (is_leaf())
      return (upper[0] - lower[0]) / 2;
    else
      return child(sc).footprint(sc);
  }

  void add_sample(vec2 sc, float flux, vec3 flux_color) {
    lock.lock();
    this->flux += flux;
    this->flux_color += flux_color;
    lock.unlock();
    if (PINE_LIKELY(!is_leaf()))
      child(sc).add_sample(sc, flux, flux_color);
    return;

    // auto x0 = psl::max(lower[0], sc[0] - footprint);
    // auto x1 = psl::min(upper[0], sc[0] + footprint);
    // auto y0 = psl::max(lower[1], sc[1] - footprint);
    // auto y1 = psl::min(upper[1], sc[1] + footprint);
    // auto weight = psl::max(x1 - x0, 0.0f) * psl::max(y1 - y0, 0.0f) * total_area;
    // lock.lock();
    // this->flux += weight * flux;
    // this->flux_color += weight * flux_color;
    // lock.unlock();
    // if (psl::abs(weight) > 0.01f && !is_leaf())
    //   for (int i = 0; i < 4; i++)
    //     child(i).add_sample(sc, footprint, total_area, flux, flux_color);
  }
  psl::optional<PgSample> sample(vec2d u, float pdf = 1) const {
    if (flux == 0.0f)
      return psl::nullopt;
    if (PINE_UNLIKELY(is_leaf())) {
      return PgSample(uniform_sphere(lerp(vec2(u), lower, upper)), pdf / (4 * Pi));
    } else {
      auto& c0 = child(0);
      auto& c2 = child(2);
      auto flux_left = c0.flux + c2.flux;
      auto rx = flux_left / flux;
      if (u[0] < rx) {
        u[0] = u[0] / rx;
        auto ry = c0.flux / flux_left;
        if (u[1] < ry) {
          u[1] = u[1] / ry;
          return c0.sample(u, pdf * 4 * rx * ry);
        } else {
          u[1] = (u[1] - ry) / (1 - ry);
          return c2.sample(u, pdf * 4 * rx * (1 - ry));
        }
      } else {
        auto& c1 = child(1);
        auto& c3 = child(3);
        u[0] = (u[0] - rx) / (1 - rx);
        auto ry = c1.flux / (c1.flux + c3.flux);
        if (u[1] < ry) {
          u[1] = u[1] / ry;
          return c1.sample(u, pdf * 4 * (1 - rx) * ry);
        } else {
          u[1] = (u[1] - ry) / (1 - ry);
          return c3.sample(u, pdf * 4 * (1 - rx) * (1 - ry));
        }
      }
    }
  }
  float pdf(vec2 sc) const {
    if (is_leaf()) {
      return 1.0f / (4 * Pi);
    } else {
      if (flux == 0.0f)
        return 0.0f;
      auto& c = child(sc);
      return 4 * c.flux / flux * c.pdf(sc);
    }
  }
  vec3 flux_estimate(vec2 sc) const {
    if (is_leaf())
      return flux_color / (area(upper - lower) * 4 * Pi);
    else
      return child(sc).flux_estimate(sc);
  }
  void clear() {
    flux = 0;
    flux_color = vec3(0);
    if (!is_leaf())
      for (int i = 0; i < 4; i++)
        child(i).clear();
  }
  int depth() const {
    return -psl::log2(upper[0] - lower[0]);
  }
  int tree_depth(vec2 sc) const {
    if (is_leaf())
      return 1;
    else
      return 1 + child(sc).tree_depth(sc);
  }

  void refine(float total_flux) {
    if (flux > total_flux * 0.01f && depth() < 15) {
      if (is_leaf()) {
        auto mid = (lower + upper) / 2;
        children = psl::array_of(QuadNode(vec2(lower[0], lower[1]), vec2(mid[0], mid[1])),
                                 QuadNode(vec2(mid[0], lower[1]), vec2(upper[0], mid[1])),
                                 QuadNode(vec2(lower[0], mid[1]), vec2(mid[0], upper[1])),
                                 QuadNode(vec2(mid[0], mid[1]), vec2(upper[0], upper[1])));
        for (int i = 0; i < 4; i++) {
          child(i).flux = flux / 4;
          child(i).flux_color = flux_color / 4;
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

private:
  bool is_leaf() const {
    return !children;
  }
  QuadNode& child(vec2 sc) {
    auto index = 0;
    index += int{sc[0] > (lower[0] + upper[0]) / 2};
    index += 2 * int{sc[1] > (lower[1] + upper[1]) / 2};
    return (*children)[index];
  }
  const QuadNode& child(vec2 sc) const {
    auto index = 0;
    index += int{sc[0] > (lower[0] + upper[0]) / 2};
    index += 2 * int{sc[1] > (lower[1] + upper[1]) / 2};
    return (*children)[index];
  }
  QuadNode& child(int index) {
    return (*children)[index];
  }
  const QuadNode& child(int index) const {
    return (*children)[index];
  }

  friend struct QuadTree;
  SpinLock lock;
  float flux = 0.0f;
  vec3 flux_color;
  vec2 lower;
  vec2 upper;
  psl::Box<psl::Array<QuadNode, 4>> children;
};

struct QuadTree {
  QuadTree() : root(vec2(0, 0), vec2(1, 1)) {
  }

  void add_sample(vec3 w, float flux, vec3 flux_color) {
    CHECK_GE(flux, 0);
    auto sc = inverse_uniform_sphere(w);
    root.add_sample(sc, flux, flux_color);
    n_samples += 1;
  }
  psl::optional<PgSample> sample(vec2d u) const {
    return root.sample(u);
  }
  float pdf(vec3 w) const {
    return root.pdf(inverse_uniform_sphere(w));
  }
  vec3 flux_estimate(vec3 w) const {
    if (n_samples == 0)
      return vec3(0.0f);
    return root.flux_estimate(inverse_uniform_sphere(w)) / int64_t(n_samples);
  }
  vec3 flux_estimate() const {
    if (n_samples == 0)
      return vec3(0.0f);
    return root.flux_color / (Pi * 4) / int64_t(n_samples);
  }
  void clear() {
    root.clear();
  }
  int tree_depth(vec3 w) const {
    return root.tree_depth(inverse_uniform_sphere(w));
  }

  void initial_refinement() {
    root.flux = 1;
    refine();
    clear();
  }
  void refine() {
    root.refine(root.flux);
  }

private:
  QuadNode root;
  Atomic<int> n_samples{0};
};

struct SpatialNode {
  SpatialNode() = default;
  SpatialNode(AABB aabb, QuadTree quad, int depth = 0)
      : aabb(aabb),
        axis(max_axis(aabb.diagonal())),
        depth(depth),
        guide(psl::move(quad)),
        collector(guide) {
  }
  psl::optional<PgSample> sample(vec2 u) const {
    return guide->sample(u);
  }
  float pdf(vec3 w) const {
    return guide->pdf(w);
  }
  vec3 flux_estimate() const {
    return guide->flux_estimate();
  }
  vec3 flux_estimate(vec3 w) const {
    return guide->flux_estimate(w);
  }
  int max_tree_depth() const {
    if (is_leaf())
      return 0;
    else
      return 1 + psl::max(child(0).max_tree_depth(), child(1).max_tree_depth());
  }

  SpatialNode& traverse(vec3 p) {
    if (is_leaf())
      return *this;
    else
      return child(p).traverse(p);
  }
  const SpatialNode& traverse(vec3 p) const {
    if (is_leaf())
      return *this;
    else
      return child(p).traverse(p);
  }

  void refine(int k) {
    if (is_leaf()) {
      const auto c = 512;
      const auto threshold = size_t(c * psl::sqrt<float>(1 << k));
      if (n_samples > threshold) {
        split_p = (aabb.lower[axis] + aabb.upper[axis]) / 2;
        auto [l_aabb, r_aabb] = aabb.split_half(axis);
        children = psl::array_of(SpatialNode(l_aabb, *collector, depth + 1),
                                 SpatialNode(r_aabb, psl::move(*collector), depth + 1));
        guide = collector = psl::nullopt;
        child(0).n_samples = n_samples / 2;
        child(1).n_samples = n_samples / 2;
        child(0).refine(k);
        child(1).refine(k);
      } else {
        collector->refine();
        guide = collector;
        collector->clear();
      }
    } else {
      child(0).refine(k);
      child(1).refine(k);
    }
    n_samples = 0;
  }
  void initial_refinement(int64_t n_samples_) {
    CHECK(is_leaf());
    n_samples = n_samples_;
    collector->initial_refinement();
    refine(0);
  }

private:
  friend struct SpatialNodeRoot;
  float footprint() {
    return aabb.diagonal()[axis] / 2;
  }
  bool is_leaf() const {
    return !children;
  }
  SpatialNode& child(int index) {
    return (*children)[index];
  }
  const SpatialNode& child(int index) const {
    return (*children)[index];
  }
  SpatialNode& child(vec3 p) {
    auto index = int{p[axis] >= split_p};
    return child(index);
  }
  const SpatialNode& child(vec3 p) const {
    auto index = int{p[axis] >= split_p};
    return child(index);
  }

  AABB aabb;
  int axis = -1;
  float split_p = -1;
  int depth = -1;
  Atomic<size_t> n_samples{0};
  psl::Box<psl::Array<SpatialNode, 2>> children;
  psl::optional<QuadTree> guide;
  psl::optional<QuadTree> collector;
};
struct SpatialNodeRoot {
  SpatialNodeRoot() = default;
  SpatialNodeRoot(AABB aabb, QuadTree quad) : root(aabb, quad) {
  }

  void add_sample(SpatialNode& leaf, vec3 p0, RadianceSample s, vec3 u) {
    CHECK_GE(s.flux[0], 0.0f);
    CHECK_GE(s.flux[1], 0.0f);
    CHECK_GE(s.flux[2], 0.0f);
    auto p = p0 + leaf.footprint() * 2 * (u - vec3(0.5f));
    for (int i = 0; i < 3; i++) {
      if (p[i] < root.aabb.lower[i])
        p[i] += 2 * (root.aabb.lower[i] - p[i]);
      else if (p[i] > root.aabb.upper[i])
        p[i] += 2 * (root.aabb.upper[i] - p[i]);
    }
    auto& chosen_leaf = traverse(p);
    chosen_leaf.n_samples += 1;
    chosen_leaf.collector->add_sample(s.w, length(s.flux), s.flux);
  }
  int max_tree_depth() const {
    return root.max_tree_depth();
  }
  SpatialNode& traverse(vec3 p) {
    return root.traverse(p);
  }
  const SpatialNode& traverse(vec3 p) const {
    return root.traverse(p);
  }
  void refine(int k) {
    return root.refine(k);
  }
  void initial_refinement(int64_t n_samples_) {
    return root.initial_refinement(n_samples_);
  }

private:
  SpatialNode root;
};

struct SpatialGrid {
  struct Unit {
    QuadTree guide, collector;
  };
  SpatialGrid() = default;
  SpatialGrid(AABB aabb, vec3i resolution) : aabb(aabb), grid(resolution) {
  }

  void add_sample(vec3 p, RadianceSample s, vec3 u) {
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
    leaf.collector.add_sample(s.w, length(s.flux), s.flux);
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
  vec3 flux_estimate(vec3 p) const {
    return traverse(p).collector.flux_estimate();
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