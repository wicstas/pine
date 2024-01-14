#pragma once

#include <pine/core/ray.h>

namespace pine {

struct Interaction {
    Ray SpawnRayTo(vec3 p2) const {
        Ray ray;
        ray.d = normalize(p2 - p, ray.tmax);
        ray.o = OffsetRayOrigin(p, face_same_hemisphere(n, ray.d));
        ray.tmin = 1e-6f;
        ray.tmax *= 1.0f - 1e-3f;
        return ray;
    }
    Ray SpawnRay(vec3 wo, float distance) const {
        Ray ray;
        ray.d = wo;
        ray.o = OffsetRayOrigin(p, face_same_hemisphere(n, ray.d));
        ray.tmin = 1e-6f;
        ray.tmax = distance * (1.0f - 1e-3f);
        return ray;
    }
    Ray SpawnRay(vec3 w) const {
        Ray ray;
        ray.d = w;
        ray.o = OffsetRayOrigin(p, face_same_hemisphere(n, ray.d));
        ray.tmin = 1e-6f;
        ray.tmax = FloatMax;
        return ray;
    }

    vec3 p;
    vec3 n;
    vec2 uv;

    const Material* material = nullptr;
    const Geometry* geometry = nullptr;
    float bvh = 0.0f;
};

}  // namespace pine

