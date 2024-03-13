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
// Skip black samples

struct RadianceSample {
  RadianceSample() = default;
  RadianceSample(vec3 w, float flux) : w{w}, flux{flux} {
  }
  vec3 w;
  float flux;
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

  float irradiance_estimate(vec2 sc) const {
    if (is_leaf()) [[unlikely]]
      return flux / sqr(length());
    else [[likely]]
      return child(sc).irradiance_estimate(sc);
  }
  float radiance_estimate(vec2 sc) const {
    if (is_leaf())
      return nsamples == 0.0f ? 0.0f : flux / nsamples;
    else
      return child(sc).radiance_estimate(sc);
  }
  void add_sample(vec2 sc, float flux) {
    this->flux += flux;
    this->nsamples += 1;
    if (!is_leaf())
      child(sc).add_sample(sc, flux);
  }
  void add_sample(vec2 sc, float flux, float half_footprint, float inv_area) {
    auto x0 = psl::max(0.0f, sc.x - half_footprint);
    auto x1 = psl::min(1.0f, sc.x + half_footprint);
    auto y0 = psl::max(0.0f, sc.y - half_footprint);
    auto y1 = psl::min(1.0f, sc.y + half_footprint);
    auto overlap = psl::max(x1 - x0, 0.0f) * psl::max(y1 - y0, 0.0f) * inv_area;
    if (overlap > 0.05f) {
      this->flux += flux * overlap;
      this->nsamples += overlap;
      if (!is_leaf()) {
        child(0).add_sample(vec2(2 * sc.x, 2 * sc.y), flux, half_footprint * 2, inv_area / 4);
        child(1).add_sample(vec2(2 * sc.x - 1, 2 * sc.y), flux, half_footprint * 2, inv_area / 4);
        child(2).add_sample(vec2(2 * sc.x, 2 * sc.y - 1), flux, half_footprint * 2, inv_area / 4);
        child(3).add_sample(vec2(2 * sc.x - 1, 2 * sc.y - 1), flux, half_footprint * 2,
                            inv_area / 4);
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
    nsamples = nsamples / 2;
    if (!is_leaf())
      for (int i = 0; i < 4; i++)
        child(i).prepare_next_iter();
  }

  void refine(float total_flux) {
    if (flux > total_flux * 0.01f && depth < 16) {
      if (is_leaf()) {
        children = psl::array_of(QuadNode(depth + 1), QuadNode(depth + 1), QuadNode(depth + 1),
                                 QuadNode(depth + 1));
        for (int i = 0; i < 4; i++) {
          child(i).flux = flux / 4;
          child(i).nsamples = nsamples / 4;
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
  Atomic<float> nsamples{0.0f};
  uint8_t depth;
  psl::Box<psl::Array<QuadNode, 4>> children;
};

struct QuadTree {
  QuadTree() : root(0) {
  }

  void add_sample(vec3 w, float flux, bool filter = true) {
    auto sc = inverse_uniform_sphere(w);
    if (filter) {
      auto footprint = root.footprint(sc);
      root.add_sample(sc, flux, footprint / 2, 1.0f / sqr(footprint));
    } else {
      root.add_sample(sc, flux);
    }
  }
  psl::optional<PgSample> sample(vec2d u) const {
    return root.sample(u);
  }
  float pdf(vec3 w) const {
    return root.pdf(inverse_uniform_sphere(w));
  }
  void prepare_next_iter() {
    root.prepare_next_iter();
  }
  void prepare_for_initial_refine() {
    root.flux = 1;
  }
  void refine() {
    root.refine(root.flux);
  }
  float radiance_estimate(vec2 sc) const {
    return root.radiance_estimate(sc);
  }
  float irradiance_estimate(vec2 sc) const {
    return root.irradiance_estimate(sc) / root.nsamples;
  }

  QuadNode root;
};

struct SpatialNode {
  SpatialNode() = default;
  SpatialNode(float footprint) : footprint_(footprint) {
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
          // no need to assign `guide`
          child(i).collector = collector;
          child(i).footprint_ = footprint_ / 2;
          child(i).n_samples = n_samples / 8;
          child(i).refine(threshold);
        }
        guide = collector = psl::nullopt;
      } else {
        collector->refine();
        guide = collector;
        collector->prepare_next_iter();
      }
    } else {
      for (int i = 0; i < 8; i++)
        child(i).refine(threshold);
    }
    n_samples = 0;
  }
  void initial_refinement(int64_t initial_samples, size_t threshold) {
    CHECK(is_leaf());
    n_samples = initial_samples;
    collector->prepare_for_initial_refine();
    refine(threshold);
  }

  float footprint() const {
    return footprint_;
  }
  float li_estimate(vec3 w) const {
    return guide->irradiance_estimate(inverse_uniform_sphere(w));
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
  psl::Box<psl::Array<SpatialNode, 8>> children;
  psl::optional<QuadTree> guide;
  psl::optional<QuadTree> collector;
};
struct SpatialTree {
  SpatialTree() = default;
  SpatialTree(AABB aabb, int64_t initial_samples, size_t threshold)
      : aabb(aabb), root(min_value(aabb.diagonal())) {
    root.collector = QuadTree();
    root.initial_refinement(initial_samples, threshold);
  }

  void add_sample(SpatialNode& leaf, vec3 p, RadianceSample s, vec3 u) {
    p += leaf.footprint() * (u - vec3(0.5f));
    p = clamp(p, aabb.lower, aabb.upper);
    auto& chosen_leaf = traverse(p);
    chosen_leaf.n_samples += 1;
    chosen_leaf.collector->add_sample(s.w, s.flux);
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

private:
  AABB aabb;
  SpatialNode root;
};

}  // namespace

}  // namespace pine