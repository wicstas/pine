#pragma once

#include <pine/core/array.h>

namespace pine {

struct Distribution1DSample {
  int i;
  float pdf = 0.0f;
};

struct Distribution1D {
  Distribution1D() = default;
  Distribution1D(const psl::vector<float>& density);

  Distribution1DSample sample(float u) const;
  float pdf(int p) const;
  float density_sum() const { return density_sum_; }

 private:
  psl::vector<float> cdf;
  float density_sum_;

 public:
  void add_point(float density) { density_builder.push_back(density); }
  void done() { *this = Distribution1D(MOVE(density_builder)); }

 private:
  psl::vector<float> density_builder;
};

struct Distribution2DSample {
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

 private:
  Node* build(const Array2d<float>& density, vec2i lower, vec2i upper, double weight,
              int depth = 0);

 public:
  Distribution2DSample sample(vec2 u2) const;
  float pdf(vec2i p) const;

  psl::shared_ptr<Node> root;

 private:
  int max_depth = 0;
};

}  // namespace pine
