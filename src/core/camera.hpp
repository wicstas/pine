#pragma once
#include <core/math.hpp>
#include <core/ray.hpp>

#include <variant>

namespace pine {

struct CameraSample {
  vec2i coordinate;
};

struct PinHoleCamera {
  PinHoleCamera(vec2i imageSize = vec2i(640, 360), float fov = Pi / 2)
      : imageSize_(imageSize), z(std::tan(fov / 2)) {
    CHECK_NE(z, 0);
  }

  Ray generateRay(CameraSample s) const {
    auto ray = Ray();
    const auto pFilm = vec2(s.coordinate - imageSize() / 2) / imageSize()[0];
    ray.d = normalize(vec3(pFilm, 0.5f / z));
    return ray;
  }

  vec2i imageSize() const { return imageSize_; }

 private:
  vec2i imageSize_;
  float z;
};

struct Camera {
  Camera() : base(PinHoleCamera()) {}
  Camera(auto base, mat4 c2w) : base(std::move(base)), c2w(c2w) {}

  Ray generateRay(CameraSample s) const {
    auto ray =
        std::visit([&](const auto& x) { return x.generateRay(s); }, base);
    ray.o = c2w * vec4(ray.o, 1);
    ray.d = mat3(c2w) * ray.d;
    DCHECK_EQF(lengthSquared(ray.d), 1);
    return ray;
  }

  vec2i imageSize() const {
    return std::visit([&](const auto& x) { return x.imageSize(); }, base);
  }

 private:
  std::variant<PinHoleCamera> base;
  mat4 c2w;
};

}  // namespace pine