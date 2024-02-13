#include <pine/core/profiler.h>
#include <pine/core/sampling.h>
#include <pine/core/geometry.h>
#include <pine/core/vecmath.h>
#include <pine/core/atomic.h>
#include <pine/core/log.h>

#include <psl/optional.h>
#include <psl/vector.h>

namespace pine {
struct RadianceSample {
  RadianceSample() = default;
  RadianceSample(vec3 p, vec3 w, vec3 flux_color)
      : p{p}, w{w}, l{length(flux_color)}, flux_color{flux_color} {
  }
  vec3 p;
  vec3 w;
  float l;
  vec3 flux_color;
};

struct PgSample {
  vec3 w;
  float pdf = 0;
};

struct QuadTreeInputSample {
  vec2 sc;
  float flux = 0;
};

struct QuadTreeSample {
  vec2 sc;
  float pdf = 0;
};

struct QuadNode;
using QuadNodes = psl::vector<QuadNode>;
struct QuadNode {
  QuadNode() = default;
  QuadNode(vec2 lower, vec2 upper) : lower{lower}, upper{upper} {
  }

  float get_footprint(const QuadNodes& nodes, vec2 sc) const {
    if (is_leaf())
      return (upper[0] - lower[0]) / 2;
    else
      return child(nodes, sc).get_footprint(nodes, sc);
  }
  vec3 flux_estimate(const QuadNodes& nodes, vec2 sc) const {
    if (is_leaf())
      return vec3{flux_color[0], flux_color[1], flux_color[2]} / area(upper - lower);
    else
      return child(nodes, sc).flux_estimate(nodes, sc);
  }

  void add_sample(QuadNodes& nodes, vec2 sc, float footprint, float inv_sq_fp, float flux,
                  vec3 flux_color) {
    CHECK_GE(flux, 0);
    this->flux += flux;
    this->flux_color[0] += flux_color[0];
    this->flux_color[1] += flux_color[1];
    this->flux_color[2] += flux_color[2];
    if (!is_leaf())
      child(nodes, sc).add_sample(nodes, sc, footprint, inv_sq_fp, flux, flux_color);
    return;

    auto x0 = psl::max(lower[0], sc[0] - footprint);
    auto x1 = psl::min(upper[0], sc[0] + footprint);
    auto y0 = psl::max(lower[1], sc[1] - footprint);
    auto y1 = psl::min(upper[1], sc[1] + footprint);
    auto weight = psl::max(x1 - x0, 0.0f) * psl::max(y1 - y0, 0.0f) * inv_sq_fp;
    this->flux += weight * flux;
    this->flux_color[0] += weight * flux_color[0];
    this->flux_color[1] += weight * flux_color[1];
    this->flux_color[2] += weight * flux_color[2];
    if (psl::abs(weight) < 0.01f)
      return;
    if (!is_leaf()) {
      for (int i = 0; i < 4; i++)
        child(nodes, i).add_sample(nodes, sc, footprint, inv_sq_fp, flux, flux_color);
    }
  }

  void refine(QuadNodes& nodes, float total_flux) {
    if (flux > total_flux * 0.01f && depth() < 15) {
      if (is_leaf()) {
        children_offset = static_cast<int>(nodes.size());
        auto mid = (lower + upper) / 2;
        nodes.emplace_back(vec2{lower[0], lower[1]}, vec2{mid[0], mid[1]});
        nodes.emplace_back(vec2{mid[0], lower[1]}, vec2{upper[0], mid[1]});
        nodes.emplace_back(vec2{lower[0], mid[1]}, vec2{mid[0], upper[1]});
        nodes.emplace_back(vec2{mid[0], mid[1]}, vec2{upper[0], upper[1]});
        for (int i = 0; i < 4; i++) {
          auto c = psl::move(child(nodes, i));
          c.flux = flux / 4;
          c.flux_color[0] = flux_color[0] / 4;
          c.flux_color[1] = flux_color[1] / 4;
          c.flux_color[2] = flux_color[2] / 4;
          c.refine(nodes, total_flux);
          child(nodes, i) = psl::move(c);
        }
      } else {
        for (int i = 0; i < 4; i++) {
          auto c = psl::move(child(nodes, i));
          c.refine(nodes, total_flux);
          child(nodes, i) = psl::move(c);
        }
      }
    } else {
      if (!is_leaf()) {
        children_offset = -1;
      }
    }
  }

  psl::optional<QuadTreeSample> sample(const QuadNodes& nodes, vec2d u, float pdf = 1) const {
    if (flux == 0.0f)
      return psl::nullopt;
    if (is_leaf()) {
      return QuadTreeSample{lerp(vec2{u}, lower, upper), pdf / (4 * Pi)};
    } else {
      auto rx = (child(nodes, 0).flux + child(nodes, 2).flux) / flux;
      if (u[0] < rx) {
        u[0] = u[0] / rx;
        auto ry = child(nodes, 0).flux / (child(nodes, 0).flux + child(nodes, 2).flux);
        if (u[1] < ry) {
          u[1] = u[1] / ry;
          return child(nodes, 0).sample(nodes, u, pdf * 4 * rx * ry);
        } else {
          u[1] = (u[1] - ry) / psl::max(1 - ry, epsilon);
          return child(nodes, 2).sample(nodes, u, pdf * 4 * rx * (1 - ry));
        }
      } else {
        u[0] = (u[0] - rx) / psl::max(1 - rx, epsilon);
        auto ry = child(nodes, 1).flux / (child(nodes, 1).flux + child(nodes, 3).flux);
        if (u[1] < ry) {
          u[1] = u[1] / ry;
          return child(nodes, 1).sample(nodes, u, pdf * 4 * (1 - rx) * ry);
        } else {
          u[1] = (u[1] - ry) / psl::max(1 - ry, epsilon);
          return child(nodes, 3).sample(nodes, u, pdf * 4 * (1 - rx) * (1 - ry));
        }
      }
    }
  }
  float pdf(const QuadNodes& nodes, vec2 sc) const {
    if (is_leaf()) {
      return 1.0f / (4 * Pi);
    } else {
      if (flux == 0.0f)
        return 0.0f;
      auto& c = child(nodes, sc);
      return 4 * c.flux / flux * c.pdf(nodes, sc);
    }
  }
  float flux_density(const QuadNodes& nodes, vec2 sc) const {
    if (flux == 0)
      return flux;
    if (is_leaf())
      return flux / (4 * Pi * area(upper - lower));
    else
      return child(nodes, sc).flux_density(nodes, sc);
  }
  bool is_leaf() const {
    return children_offset == -1;
  }
  int depth() const {
    return -psl::log2(upper[0] - lower[0]);
  }
  int tree_depth(const QuadNodes& nodes, vec2 sc) const {
    if (is_leaf())
      return 1;
    else
      return 1 + child(nodes, sc).tree_depth(nodes, sc);
  }
  QuadNode& child(QuadNodes& nodes, vec2 sc) const {
    auto index = 0;
    index += int{sc[0] > (lower[0] + upper[0]) / 2};
    index += 2 * int{sc[1] > (lower[1] + upper[1]) / 2};
    return child(nodes, index);
  }
  const QuadNode& child(const QuadNodes& nodes, vec2 sc) const {
    auto index = 0;
    index += int{sc[0] > (lower[0] + upper[0]) / 2};
    index += 2 * int{sc[1] > (lower[1] + upper[1]) / 2};
    return child(nodes, index);
  }
  QuadNode& child(QuadNodes& nodes, int index) const {
    return nodes[children_offset + index];
  }
  const QuadNode& child(const QuadNodes& nodes, int index) const {
    return nodes[children_offset + index];
  }

  Atomic<float> flux{0};
  Atomic<float> flux_color[3]{0, 0, 0};
  vec2 lower;
  vec2 upper;
  int children_offset = -1;
};

struct QuadTree {
  QuadTree() {
    nodes.push_back(QuadNode{vec2{0, 0}, vec2{1, 1}});
  }
  float get_footprint(vec2 sc) const {
    return root().get_footprint(nodes, sc);
  }
  vec3 flux_estimate(vec2 sc) const {
    if (n_samples == 0)
      return vec3{0.0f};
    return root().flux_estimate(nodes, sc) / int{n_samples};
  }

  void add_sample(vec2 sc, float flux, vec3 flux_color) {
    CHECK_GE(flux, 0);
    auto fp = get_footprint(sc);
    root().add_sample(nodes, sc, fp, 1.0f / psl::sqr(fp), flux, flux_color);
    n_samples += 1;
  }

  void refine() {
    auto c = psl::move(root());
    c.refine(nodes, root().flux);
    root() = psl::move(c);
  }

  psl::optional<QuadTreeSample> sample(vec2d u) const {
    return root().sample(nodes, u);
  }
  float pdf(vec2 sc) const {
    return root().pdf(nodes, sc);
  }
  void clear() {
    for (auto& node : nodes) {
      node.flux = 0;
      node.flux_color[0] = 0;
      node.flux_color[1] = 0;
      node.flux_color[2] = 0;
    }
    n_samples = 0;
  }
  int tree_depth(vec2 sc) const {
    return root().tree_depth(nodes, sc);
  }

  QuadNode& root() {
    return nodes[0];
  }
  const QuadNode& root() const {
    return nodes[0];
  }

  QuadNodes nodes;
  Atomic<int> n_samples{0};
};

struct SpatialNode;
using SpatialNodes = psl::vector<SpatialNode>;

struct SpatialNode {
  SpatialNode() = default;
  SpatialNode(AABB aabb, QuadTree quad)
      : aabb{aabb}, axis{max_axis(aabb.diagonal())}, guide{psl::move(quad)}, collector{guide} {
  }

  float get_footprint() {
    return aabb.diagonal()[axis] / 2;
  }
  SpatialNode& add_sample(RadianceSample s) {
    CHECK_GE(s.l, 0);
    n_samples += 1;
    collector->add_sample(inverse_uniform_sphere(s.w), s.l, s.flux_color);
    return *this;
  }
  SpatialNode& refine(SpatialNodes& nodes, int k) {
    if (is_leaf()) {
      const auto c = 2000;
      const auto threshold = static_cast<size_t>(c * psl::sqrt(psl::sqrt<float>(1 << k)));
      if (n_samples > threshold) {
        split_p = psl::lerp(0.5f, aabb.lower[axis], aabb.upper[axis]);
        auto [l_aabb, r_aabb] = aabb.split_half(axis);
        children_indices[0] = static_cast<int>(nodes.size()) + 0;
        children_indices[1] = static_cast<int>(nodes.size()) + 1;
        nodes.emplace_back(l_aabb, *collector);
        nodes.emplace_back(r_aabb, psl::move(*collector));
        guide = collector = psl::nullopt;
        auto c0 = psl::move(child(nodes, 0));
        auto c1 = psl::move(child(nodes, 1));
        c0.n_samples = n_samples / 2;
        c1.n_samples = n_samples / 2;
        c0.refine(nodes, k);
        c1.refine(nodes, k);
        child(nodes, 0) = psl::move(c0);
        child(nodes, 1) = psl::move(c1);
      } else {
        Profiler _("Quad refine");
        collector->refine();
        guide = collector;
        collector->clear();
      }
      n_samples = 0;
    } else {
      auto c0 = psl::move(nodes[children_indices[0]]);
      auto c1 = psl::move(nodes[children_indices[1]]);
      c0.refine(nodes, k);
      c1.refine(nodes, k);
      nodes[children_indices[0]] = psl::move(c0);
      nodes[children_indices[1]] = psl::move(c1);
    }
    return *this;
  }

  psl::optional<PgSample> sample(vec2 u) const {
    CHECK(is_leaf());
    if (auto qs = guide->sample(u)) {
      return PgSample{uniform_sphere(qs->sc), qs->pdf};
    } else
      return psl::nullopt;
  }
  float pdf(vec3 w) const {
    CHECK(is_leaf());
    auto sc = inverse_uniform_sphere(w);
    return guide->pdf(sc);
  }

  SpatialNode& traverse(SpatialNodes& nodes, vec3 p) {
    if (is_leaf())
      return *this;
    else
      return child(nodes, p).traverse(nodes, p);
  }
  const SpatialNode& traverse(const SpatialNodes& nodes, vec3 p) const {
    if (is_leaf())
      return *this;
    else
      return child(nodes, p).traverse(nodes, p);
  }

  bool is_leaf() const {
    return children_indices[0] == -1;
    if (children_indices[0] != -1) {
      CHECK(children_indices[1] != -1);
      CHECK(!guide);
      CHECK(!collector);
      return false;
    } else {
      CHECK(children_indices[1] == -1);
      CHECK(guide);
      CHECK(collector);
      return true;
    }
  }

  SpatialNode& child(SpatialNodes& nodes, int index) {
    CHECK_RANGE(index, 0, 1);
    return nodes[children_indices[index]];
  }
  const SpatialNode& child(const SpatialNodes& nodes, int index) const {
    CHECK_RANGE(index, 0, 1);
    return nodes[children_indices[index]];
  }
  SpatialNode& child(SpatialNodes& nodes, vec3 p) {
    auto index = int{p[axis] >= split_p};
    return child(nodes, index);
  }
  const SpatialNode& child(const SpatialNodes& nodes, vec3 p) const {
    auto index = int{p[axis] >= split_p};
    return child(nodes, index);
  }

  AABB aabb;
  int axis;
  float split_p;
  Atomic<size_t> n_samples{0};
  psl::optional<QuadTree> guide;
  psl::optional<QuadTree> collector;
  int children_indices[2]{-1, -1};
};

struct SpatialTree {
  SpatialTree() = default;
  SpatialTree(AABB top_aabb, QuadTree quad_tree) {
    nodes.push_back(SpatialNode{top_aabb, quad_tree});
  }
  void add_sample(SpatialNode& leaf, RadianceSample s, vec3 up) {
    CHECK_GE(s.l, 0.0f);
    up -= vec3{0.5f};
    s.p += leaf.get_footprint() * 2 * up;
    auto aabb = root().aabb;
    for (int i = 0; i < 3; i++) {
      if (s.p[i] < aabb.lower[i])
        s.p[i] += 2 * (aabb.lower[i] - s.p[i]);
      else if (s.p[i] > aabb.upper[i])
        s.p[i] += 2 * (aabb.upper[i] - s.p[i]);
    }
    traverse(s.p).add_sample(s);
  }
  void refine(int k) {
    root() = SpatialNode{root()}.refine(nodes, k);
  }
  SpatialNode& traverse(vec3 p) {
    return root().traverse(nodes, p);
  }
  SpatialNode& root() {
    return nodes[0];
  }
  const SpatialNode& root() const {
    return nodes[0];
  }

private:
  SpatialNodes nodes;
};

}  // namespace pine