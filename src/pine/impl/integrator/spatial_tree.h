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
  void add_sample(vec2 sc, float flux, float half_footprint, float inv_area,
                  bool terminate_recursion) {
    this->flux += flux;
    if (!terminate_recursion && !is_leaf()) {
      auto i = 0;
      for (float y = 0; y < 1; y += 0.5f) {
        for (float x = 0; x < 1; x += 0.5f) {
          auto x0 = psl::max(x, sc.x - half_footprint);
          auto x1 = psl::min(x + 0.5f, sc.x + half_footprint);
          auto y0 = psl::max(y, sc.y - half_footprint);
          auto y1 = psl::min(y + 0.5f, sc.y + half_footprint);
          auto overlap = psl::max(x1 - x0, 0.0f) * psl::max(y1 - y0, 0.0f) * inv_area;
          auto c_sc = vec2(2 * sc.x - x * 2, 2 * sc.y - y * 2);
          child(i++).add_sample(c_sc, flux * overlap, half_footprint, inv_area, overlap < 0.05f);
        }
      }
    }
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
  void prepare_next_iter() {
    // Moving average
    flux = flux / 2;
    if (!is_leaf())
      for (int i = 0; i < 4; i++)
        child(i).prepare_next_iter();
  }

  void refine(float total_flux) {
    if (flux > total_flux * 0.015f && depth < 16) {
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
  float length() const {
    return 1.0f / (1 << depth);
  }

  float flux_density(vec2 sc) const {
    if (is_leaf())
      return flux / sqr(length());
    else
      return child(sc).flux_density(sc);
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

  void add_sample(vec3 w, float flux) {
    if (flux == 0.0f)
      return;
    auto sc = inverse_uniform_sphere(w);
    auto footprint = root.footprint(sc);
    root.add_sample(sc, flux, footprint / 2, 1.0f / sqr(footprint), false);
    // n_samples += 1;
  }
  psl::optional<PgSample> sample(vec2d u) const {
    return root.sample(u);
  }
  float pdf(vec3 w) const {
    return root.pdf(inverse_uniform_sphere(w));
  }
  void prepare_next_iter() {
    // n_samples = n_samples / 2;
    root.prepare_next_iter();
  }
  void prepare_for_initial_refine() {
    root.flux = 1;
  }
  void refine() {
    root.refine(root.flux);
  }
  float flux_density(vec2) const {
    return 0.0f;
    // return root.flux_density(sc) / n_samples;
  }

  QuadNode root;
  // Atomic<int> n_samples{0};
};

struct DirectionalBin {
  float splitting_factor(vec3 throughput, float g_cost_to_var) const {
    auto n = 1.0f;
    auto split_s = average(psl::sqr(throughput) * var_to_cost) * g_cost_to_var;
    auto split_r = average(psl::sqr(throughput) * moment2_to_cost) * g_cost_to_var;
    if (split_r > 1) {
      if (split_s > 1)
        n = split_s;
    } else {
      n = split_r;
    }
    return psl::sqrt(n);
  }
  Vector3<Atomic<float>> estimate, moment2;
  Atomic<float> cost{0};
  Atomic<uint32_t> n{0};
  bool valid = false;
  vec3 var_to_cost, moment2_to_cost;
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
  DirectionalBin& bin_of(vec3 w) {
    auto sc = inverse_uniform_sphere(w);
    auto x = psl::min(int(sc.x * bin_resolution), bin_resolution - 1);
    auto y = psl::min(int(sc.y * bin_resolution), bin_resolution - 1);
    return bins[x + y * bin_resolution];
  }
  void refine(size_t threshold) {
    if (is_leaf()) {
      collector->refine();
      if (n_samples > threshold) {
        children = children.default_value();
        auto decay_factor = 1;
        for (auto& bin : bins) {
          bin.cost = bin.cost / 8 / decay_factor;
          bin.estimate = bin.estimate / 8 / decay_factor;
          bin.moment2 = bin.moment2 / 8 / decay_factor;
          bin.n = bin.n / 8 / decay_factor;
        }
        for (int i = 0; i < 8; i++) {
          child(i).bins = bins;
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
        guide = collector;
        collector->prepare_next_iter();
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
    this->weight_a = this->weight_a * 0.5f;
    this->weight_b = this->weight_b * 0.5f;
    this->alpha_a = this->alpha_a * 0.5f;
    this->alpha_b = this->alpha_b * 0.5f;
  }
  void initial_refinement(int64_t initial_samples, size_t threshold) {
    CHECK(is_leaf());
    n_samples = initial_samples;
    collector->prepare_for_initial_refine();
    refine(threshold);
    n_samples = 0;
  }

  float footprint() const {
    return footprint_;
  }
  float strategy_a_prob() const {
    return prob_a;
  }
  float flux_density(vec3 w) const {
    return guide->flux_density(inverse_uniform_sphere(w));
  }

  void for_each_bin(auto f) {
    if (is_leaf()) {
      for (auto& bin : bins)
        f(bin);
    } else {
      for (int i = 0; i < 8; i++)
        child(i).for_each_bin(f);
    }
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
  static constexpr int bin_resolution = 4;
  psl::Array<DirectionalBin, bin_resolution * bin_resolution> bins;
  psl::optional<QuadTree> guide;
  psl::optional<QuadTree> collector;
};
struct SpatialTree {
  SpatialTree() = default;
  SpatialTree(AABB aabb, int64_t initial_samples, size_t threshold)
      : aabb(aabb), root(min_value(aabb.diagonal()), QuadTree()) {
    root.initial_refinement(initial_samples, threshold);
  }

  void add_sample(SpatialNode& leaf, vec3 p, RadianceSample s, vec3 u) {
    p += leaf.footprint() * (u - vec3(0.5f));
    p = clamp(p, aabb.lower, aabb.upper);
    auto& chosen_leaf = traverse(p);
    chosen_leaf.n_samples += 1;
    chosen_leaf.collector->add_sample(s.w, s.flux);
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
  void for_each_bin(auto f) {
    root.for_each_bin(f);
  }

private:
  AABB aabb;
  SpatialNode root;
};

}  // namespace

}  // namespace pine