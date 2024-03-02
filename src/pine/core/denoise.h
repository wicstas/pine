#pragma once
#include <pine/core/array.h>

namespace pine {

enum DenoiseQuality { Medium, High };
void denoise(DenoiseQuality quality, Array2d<vec3>& output, const Array2d<vec3>& color,
             const Array2d<vec3>* albedo = nullptr, const Array2d<vec3>* normal = nullptr);

inline void denoise(DenoiseQuality quality, Array2d<vec3>& output, const Array2d<vec3>& color,
                    const Array2d<vec3>& albedo, const Array2d<vec3>& normal) {
  denoise(quality, output, color, &albedo, &normal);
}

}  // namespace pine
