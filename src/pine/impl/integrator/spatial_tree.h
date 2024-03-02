#include <pine/core/sampling.h>
#include <pine/core/geometry.h>
#include <pine/core/atomic.h>
#include <pine/core/array.h>

#include <psl/optional.h>
#include <psl/vector.h>
#include <psl/array.h>

namespace pine {

namespace {

// TODO prune nodes

struct RadianceSample {
  RadianceSample() = default;
  RadianceSample(vec3 w, float flux, psl::optional<float> weight_a, psl::optional<float> weight_b)
      : w{w}, flux{flux}, weight_a(weight_a), weight_b(weight_b) {
  }
  vec3 w;
  float flux;
  psl::optional<float> weight_a, weight_b;
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
    if (flux > total_flux * 0.02f && depth < 16) {
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
  uint8_t depth;
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
  SpatialNode(float footprint, QuadTree quad)
      : footprint_(footprint), guide(quad), collector(psl::move(quad)) {
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

  void refine(size_t threshold) {
    if (is_leaf()) {
      if (n_samples > threshold) {
        children = children.default_value();
        for (int i = 0; i < 8; i++) {
          // no need to assign guide
          child(i).collector = collector;
          child(i).footprint_ = footprint_ / 2;
          child(i).n_samples = n_samples / 8;
          child(i).weight_a = weight_a / 8;
          child(i).weight_b = weight_b / 8;
          child(i).alpha_a = alpha_a / 8;
          child(i).alpha_b = alpha_b / 8;
          child(i).refine(threshold);
        }
        guide = collector = psl::nullopt;
      } else {
        collector->refine();
        guide = collector;
        collector->clear();
      }
    } else {
      for (int i = 0; i < 8; i++)
        child(i).refine(threshold);
    }
    n_samples = 0;
    prob_a = 0.5f;
    auto weight_a = float(this->weight_a);
    auto weight_b = float(this->weight_b);
    auto alpha_a = int(this->alpha_a);
    auto alpha_b = int(this->alpha_b);
    if (alpha_a > 100 && alpha_b > 100) {
      weight_a /= alpha_a;
      weight_b /= alpha_b;
      auto weight_sum = weight_a + weight_b;
      if (weight_sum > 0)
        prob_a = weight_a / weight_sum;
    }
    // weight_a = 0;
    // weight_b = 0;
    // alpha_a = 0;
    // alpha_b = 0;
  }
  void initial_refinement(int64_t initial_samples, size_t threshold) {
    CHECK(is_leaf());
    n_samples = initial_samples;
    collector->initial_refinement();
    refine(threshold);
    n_samples = 0;
  }

  float footprint() const {
    return footprint_;
  }
  float strategy_a_prob() const {
    return prob_a;
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
  friend struct SpatialTree;
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
    auto index = 0;
#pragma unroll
    for (int i = 0; i < 3; i++) {
      if (p[i] < 0.5f) {
        p[i] = p[i] * 2;
      } else {
        p[i] = (p[i] - 0.5f) * 2;
        index += 1 << i;
      }
    }
    return child(index);
  }
  const SpatialNode& child(vec3& p) const {
    return const_cast<SpatialNode*>(this)->child(p);
  }

  float footprint_ = 0.0f;
  Atomic<size_t> n_samples{0};
  Atomic<float> weight_a{0}, weight_b{0};
  Atomic<int> alpha_a{0}, alpha_b{0};
  float prob_a = 0.5f;
  psl::Box<psl::Array<SpatialNode, 8>> children;
  psl::optional<QuadTree> guide;
  psl::optional<QuadTree> collector;
};
struct SpatialTree {
  SpatialTree() = default;
  SpatialTree(AABB aabb, int64_t initial_samples, size_t threshold)
      : aabb(aabb), root(min_value(aabb.diagonal()), QuadTree()) {
    root.initial_refinement(initial_samples, threshold);
  }

  void add_sample(SpatialNode& leaf, vec3 p, RadianceSample s, vec3 u, vec2 ud) {
    p += leaf.footprint() * (u - vec3(0.5f));
    p = clamp(p, aabb.lower, aabb.upper);
    auto& chosen_leaf = traverse(p);
    chosen_leaf.n_samples += 1;
    chosen_leaf.collector->add_sample(s.w, s.flux, ud);
    if (s.weight_a) {
      chosen_leaf.weight_a += *s.weight_a;
      chosen_leaf.alpha_a += 1;
    } else {
      chosen_leaf.weight_b += *s.weight_b;
      chosen_leaf.alpha_b += 1;
    }
  }
  SpatialNode& traverse(vec3 p) {
    return root.traverse(aabb.relative_position(p));
  }
  const SpatialNode& traverse(vec3 p) const {
    return root.traverse(aabb.relative_position(p));
  }
  void refine(size_t threshold) {
    return root.refine(threshold);
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

}  // namespace

}  // namespace pine