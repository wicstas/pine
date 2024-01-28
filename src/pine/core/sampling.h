#pragma once

#include <pine/core/vecmath.h>

namespace pine {

struct SpatialPdf {
    operator float() const {
        return pos * dir;
    }
    float pos = 0.0f;
    float dir = 0.0f;
};

inline vec2 sample_disk_polar(vec2 u) {
    float r = psl::sqrt(u[0]);
    float theta = 2 * pi * u[1];
    return {r * psl::cos(theta), r * psl::sin(theta)};
}

inline vec2 sample_disk_concentric(vec2 u) {
    u = vec2(u.x * 2 - 1.0f, u.y * 2 - 1.0f);
    float theta, r;
    if (psl::abs(u.x) > psl::abs(u.y)) {
        r = u.x;
        theta = pi / 4.0f * u.y / u.x;
    } else {
        r = u.y;
        theta = pi / 2.0f - pi / 4.0f * (u.x / u.y);
    }
    return r * vec2(psl::cos(theta), psl::sin(theta));
}

inline vec3 cosine_weighted_hemisphere(vec2 u) {
    auto d = sample_disk_concentric(u);
    auto z = psl::sqrt(psl::max(1.0f - d.x * d.x - d.y * d.y, 0.0f));
    return vec3(d.x, d.y, z);
}

inline vec3 uniform_sphere(vec2 u) {
    return spherical_to_cartesian(u.x * pi * 2, psl::acos(1.0f - 2 * u.y));
}
inline vec2 inverse_uniform_sphere(vec3 d) {
    auto [phi, theta] = cartesian_to_spherical(d);
    return {phi / pi2, (1.0f - psl::cos(theta)) / 2.0f};
}

inline vec3 uniform_hemisphere(vec2 u) {
    return spherical_to_cartesian(u.x * pi * 2, psl::acos(u.y));
}
inline vec2 inverse_uniform_hemisphere(vec3 d) {
    auto [phi, theta] = cartesian_to_spherical(d);
    return {phi / pi2, psl::cos(theta)};
}

inline float balance_heuristic(int nF, float pF, int nG, float pG) {
    return nF * pF / (nF * pF + nG * pG);
}
inline float power_heuristic(int nF, float pF, int nG, float pG) {
    float f = nF * pF;
    float g = nG * pG;
    return psl::sqr(f) / (psl::sqr(f) + psl::sqr(g));
}

}  // namespace pine


