#pragma once
#include <core/camera.hpp>
#include <core/shapes.hpp>

#include <vector>

namespace pine {

struct Scene {
  void setCamera(auto... args) { camera = Camera(args...); }
  void addShape(auto... args) { shapes.push_back(Shape(args...)); }

  Camera& getCamera() { return camera; }

  bool intersect(Ray& ray, Intersection& it) const {
    auto hit = false;
    for (const auto& shape : shapes) hit |= shape.intersect(ray, it);
    return hit;
  }

 private:
  Camera camera;
  std::vector<Shape> shapes;
};

}  // namespace pine
