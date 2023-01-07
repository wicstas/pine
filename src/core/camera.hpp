#pragma once
#include <core/image.hpp>
#include <core/color.hpp>
#include <core/math.hpp>
#include <core/ray.hpp>

#include <variant>

namespace pine {

struct CameraSample {
  vec2i coordinate;
};

struct PinHoleCamera {
  PinHoleCamera() = default;
  PinHoleCamera(vec2i imageSize, vec3 lookFrom, vec3 lookAt, float fov)
      : image(imageSize),
        lookFrom(lookFrom),
        c2w(pine::lookAt(lookFrom, lookAt)),
        z(0.5f / std::tan(fov / 2)) {
    CHECK_NE(z, 0);
  }

  Ray generateRay(CameraSample s) const {
    vec2 pFilm = vec2(s.coordinate - image.size() / 2) / image.width();
    const auto dir = normalize(c2w * vec3(pFilm, z));
    return Ray(lookFrom, dir);
  }
  void recordColor(CameraSample s, Spectrum color) {
    image[s.coordinate] = color.toRGB();
  }

  vec2i imageSize() const { return image.size(); }

  Image<vec3>& getImage() { return image; }

 private:
  Image<vec3> image;
  vec3 lookFrom;
  mat3 c2w;
  float z;
};

struct Camera {
  Camera() : base(PinHoleCamera()) {}
  Camera(auto base) : base(std::move(base)) {}

  Ray generateRay(CameraSample s) const {
    auto ray = std::visit([&](auto&& x) { return x.generateRay(s); }, base);
    DCHECK_EQF(lengthSquared(ray.d), 1);
    return ray;
  }

  void recordColor(CameraSample s, Spectrum color) {
    return std::visit([&](auto&& x) { return x.recordColor(s, color); }, base);
  }

  vec2i imageSize() const {
    return std::visit([&](auto&& x) { return x.imageSize(); }, base);
  }

  Image<vec3>& getImage() {
    return std::visit([&](auto&& x) -> decltype(auto) { return x.getImage(); }, base);
  }

 private:
  std::variant<PinHoleCamera> base;
};

}  // namespace pine