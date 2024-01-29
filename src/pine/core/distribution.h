#pragma once

#include <pine/core/array.h>

namespace pine {

struct DistributionSample {
  vec2i p;
  float pdf = 0.0f;
};

struct Distribution2D {
  struct Node {
    float weight = 0.0f;
    bool split_x = true;
    vec2i lower;
    vec2i upper;
    psl::unique_ptr<Node> left;
    psl::unique_ptr<Node> right;
  };

  Distribution2D() = default;
  Distribution2D(const Array2d<float>& density, int max_depth);

  Node* build(const Array2d<float>& density, vec2i lower, vec2i upper, double weight, int depth = 0);

  DistributionSample sample(vec2 u2) const;
  float pdf(vec2i p) const;
  float density_to_pdf() const {
    return density_to_pdf_;
  }

  psl::shared_ptr<Node> root;

private:
  int max_depth = 6;
  float density_to_pdf_ = 0.0;
};

}  // namespace pine
