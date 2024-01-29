#include <pine/core/distribution.h>
#include <pine/core/profiler.h>

namespace pine {

Distribution2D::Distribution2D(const Array2d<float>& density, int max_depth)
    : max_depth{max_depth} {
  Profiler _("Build Distribution2D");
  auto weight = 0.0;
  for_2d(density.size(), [&](auto p) {
    CHECK_GE(density[p], 0);
    weight += density[p];
  });
  density_to_pdf_ = weight / area(density.size());
  root = psl::shared_ptr<Node>{build(density, vec2i{0}, density.size(), weight)};
}

Distribution2D::Node* Distribution2D::build(const Array2d<float>& density, vec2i lower, vec2i upper,
                                            double weight, int depth) {
  auto size = upper - lower;

  if (depth >= max_depth || size.x == 0 || size.y == 0 || size == vec2i{1})
    return new Node{static_cast<float>(weight), true, lower, upper, {}, {}};

  auto partial_weight = 0.0;
  auto split_x = size.x >= size.y;
  auto p = 0;

  if (split_x) {
    for (int x = lower.x; x < upper.x - 1; x++) {
      auto sum = 0.0;
      for (int y = lower.y; y < upper.y; y++)
        sum += density[{x, y}];
      partial_weight += sum;
      if (partial_weight >= weight / 2) {
        p = x + 1;
        x = upper.x;
        break;
      }
    }
  } else {
    for (int y = lower.y; y < upper.y - 1; y++) {
      auto sum = 0.0;
      for (int x = lower.x; x < upper.x; x++)
        sum += density[{x, y}];
      partial_weight += sum;
      if (partial_weight >= weight / 2) {
        p = y + 1;
        y = upper.y;
        break;
      }
    }
  }

  if (p == 0)
    p = split_x ? upper.x - 1 : upper.y - 1;

  return new Node{
      static_cast<float>(weight),
      split_x,
      lower,
      upper,
      psl::unique_ptr<Node>{build(density, lower, split_x ? vec2i{p, upper.y} : vec2i{upper.x, p},
                                  partial_weight, depth + 1)},
      psl::unique_ptr<Node>{build(density, split_x ? vec2i{p, lower.y} : vec2i{lower.x, p}, upper,
                                  weight - partial_weight, depth + 1)}};
}

static DistributionSample sample(const Distribution2D::Node* node, vec2 u2, float pdf) {
  if (!node->left || node->weight == 0.0f)
    return DistributionSample{lerp(u2, node->lower, node->upper),
                              pdf / area(node->upper - node->lower)};

  auto r = node->left->weight / node->weight;
  CHECK_RANGE(r, 0, 1);
  auto& u = node->split_x ? u2[0] : u2[1];
  if (u <= r) {
    u = u / psl::max(r, epsilon);
    return sample(node->left.get(), u2, pdf * r);
  } else {
    u = (u - r) / (1 - r);
    return sample(node->right.get(), u2, pdf * (1 - r));
  }
}

static float pdf(const Distribution2D::Node* node, vec2i p, float pdf_) {
  if (node->weight == 0)
    return 0;
  if (!node->left)
    return pdf_ / area(node->upper - node->lower);

  if (inside(p, node->left->lower, node->left->upper))
    return pdf(node->left.get(), p, pdf_ * node->left->weight / node->weight);
  else
    return pdf(node->right.get(), p, pdf_ * node->right->weight / node->weight);
}

DistributionSample Distribution2D::sample(vec2 u2) const {
  if (!root)
    return DistributionSample{};

  return pine::sample(root.get(), u2, area(root->upper - root->lower));
}

float Distribution2D::pdf(vec2i p) const {
  return pine::pdf(root.get(), p, area(root->upper - root->lower));
}

}  // namespace pine