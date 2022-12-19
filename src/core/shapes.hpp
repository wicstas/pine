#pragma once
#include <core/intersection.hpp>
#include <core/ray.hpp>

#include <optional>
#include <variant>

namespace pine {

struct Sphere {
  Sphere(vec3 p, float radius) : p(p), radius(radius) {}

  bool intersect(Ray& ray, Intersection& it) const {
    const auto a = dot(ray.d, ray.d);
    const auto b = 2 * dot(ray.o - p, ray.d);
    const auto c = dot(ray.o - p, ray.o - p) - sqr(radius);
    const auto determinant = sqr(b) - 4 * a * c;
    if (determinant <= 0) return false;

    auto t = (-b - std::sqrt(determinant)) / (2 * a);
    if (ray.tooClose(t)) t = (-b + std::sqrt(determinant)) / (2 * a);

    if (!ray.contains(t)) return false;

    ray.updateTmax(t);
    it.p = ray(t);
    it.n = normalize(it.p - p);
    return true;
  }

 private:
  vec3 p;
  float radius;
};

struct Plane {
  Plane(vec3 p, vec3 n) : p(p), n(normalize(n)) {}

  bool intersect(Ray& ray, Intersection& it) const {
    const auto t = -dot(ray.o - p, n) / dot(ray.d, n);
    if (!ray.contains(t)) return false;

    ray.updateTmax(t);
    it.p = ray(t);
    it.n = n;
    return true;
  }

 private:
  vec3 p;
  vec3 n;
};

struct Shape {
  Shape(auto base) : base(std::move(base)) {}

  bool intersect(Ray& ray, Intersection& it) const {
    return std::visit([&](const auto& x) { return x.intersect(ray, it); },
                      base);
  }

 private:
  std::variant<Sphere, Plane> base;
};

}  // namespace pine
