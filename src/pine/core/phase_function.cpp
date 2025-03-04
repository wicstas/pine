#include <pine/core/phase_function.h>
#include <pine/core/sampling.h>

namespace pine {

float eval_henyey_greenstein(float cos_theta, float g) {
  auto denom = 1 + sqr(g) + 2 * g * cos_theta;
  return (1 - sqr(g)) / (denom * psl::safe_sqrt(denom) * Pi * 4);
}
float sample_henyey_greenstein(float xi, float g) {
  if (std::abs(g) < 1e-3f)
    return 1 - 2 * xi;
  else
    return -1 / (2 * g) * (1 + sqr(g) - sqr((1 - sqr(g)) / (1 + g - 2 * g * xi)));
}

/*
 * SPDX-FileCopyrightText: Copyright (c) <2023> NVIDIA CORPORATION & AFFILIATES. All rights
 * reserved. SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

// [Jendersie and d'Eon 2023]
//   SIGGRAPH 2023 Talks
//   https://doi.org/10.1145/3587421.3595409

// EVAL and SAMPLE for the Draine (and therefore Cornette-Shanks) phase function
//   g = HG shape parameter
//   a = "alpha" shape parameter

// WARNING: these functions don't special case isotropic scattering and can numerically fail for
// certain inputs

// eval:
//   u = dot(prev_dir, next_dir)
float eval_draine(float u, float g, float a) {
  return ((1 - g * g) * (1 + a * u * u)) /
         (4.0f * (1 + (a * (1 + 2 * g * g)) / 3.0f) * Pi * psl::pow(1 + g * g - 2 * g * u, 1.5f));
}

// sample: (sample an exact deflection cosine)
//   xi = a uniform random real [0,1]
float sample_draine_cos(float xi, float g, float a) {
  const float g2 = g * g;
  const float g3 = g * g2;
  const float g4 = g2 * g2;
  const float g6 = g2 * g4;
  const float pgp1_2 = (1 + g2) * (1 + g2);
  //   const float T1 = (-1 + g2) * (4 * g2 + a * pgp1_2);
  const float T1a = -a + a * g4;
  const float T1a3 = T1a * T1a * T1a;
  const float T2 = -1296 * (-1 + g2) * (a - a * g2) * (T1a) * (4 * g2 + a * pgp1_2);
  const float T3 =
      3 * g2 * (1 + g * (-1 + 2 * xi)) + a * (2 + g2 + g3 * (1 + 2 * g2) * (-1 + 2 * xi));
  const float T4a = 432 * T1a3 + T2 + 432 * (a - a * g2) * T3 * T3;
  const float T4b = -144 * a * g2 + 288 * a * g4 - 144 * a * g6;
  const float T4b3 = T4b * T4b * T4b;
  const float T4 = T4a + psl::sqrt(-4 * T4b3 + T4a * T4a);
  const float T4p3 = psl::pow(T4, 1.0f / 3.0f);
  const float T6 =
      (2 * T1a + (48 * psl::pow(2.0f, 1.0f / 3.0f) * (-(a * g2) + 2 * a * g4 - a * g6)) / T4p3 +
       T4p3 / (3.0f * psl::pow(2.0f, 1.0f / 3.0f))) /
      (a - a * g2);
  const float T5 = 6 * (1 + g2) + T6;
  return (1 + g2 -
          psl::sqr(-0.5f * psl::sqrt(T5) +
                   psl::sqrt(6 * (1 + g2) - (8 * T3) / (a * (-1 + g2) * psl::sqrt(T5)) - T6) /
                       2.0f)) /
         (2.0f * g);
}

PhaseFunctionSample TwoLobeHgPhaseFunction::sample(vec3 wi, vec2 u) const {
  auto ps = PhaseFunctionSample();
  float cos_theta;
  if (with_prob(w, u[0]))
    cos_theta = sample_henyey_greenstein(u[0], g1);
  else
    cos_theta = sample_henyey_greenstein(u[0], g0);
  ps.f = psl::lerp(w, eval_henyey_greenstein(cos_theta, g0), eval_henyey_greenstein(cos_theta, g1));
  ps.pdf = ps.f;

  auto sin_theta = psl::safe_sqrt(1 - sqr(cos_theta));
  auto phi = 2 * Pi * u[1];
  ps.wo = coordinate_system(wi) * spherical_to_cartesian(phi, sin_theta, cos_theta);

  return ps;
}

float TwoLobeHgPhaseFunction::f(vec3 wi, vec3 wo) const {
  auto cos_theta = dot(wi, wo);
  return psl::lerp(w, eval_henyey_greenstein(cos_theta, g0), eval_henyey_greenstein(cos_theta, g1));
}

PhaseFunctionSample HgPhaseFunction::sample(vec3 wi, vec2 u) const {
  auto ps = PhaseFunctionSample();
  auto cos_theta = sample_henyey_greenstein(u[0], g);

  auto sin_theta = psl::safe_sqrt(1 - sqr(cos_theta));
  auto phi = 2 * Pi * u[1];
  auto m = coordinate_system(wi);
  ps.wo = m * spherical_to_cartesian(phi, sin_theta, cos_theta);
  ps.pdf = eval_henyey_greenstein(cos_theta, g);
  ps.f = ps.pdf;
  return ps;
}

CloudPhaseFunction::CloudPhaseFunction(float d) {
  g_hg = psl::exp(-0.0990567f / (d - 1.67154f));
  g_d = psl::exp(-2.20679f / (d + 3.91029f) - 0.428934f);
  a = psl::exp(3.62489f - 8.29288f / (d + 5.52825f));
  w = psl::exp(-0.599085f / (d - 0.641583f) - 0.665888f);
}
PhaseFunctionSample CloudPhaseFunction::sample(vec3 wi, vec2 u) const {
  auto ps = PhaseFunctionSample();
  float cos_theta;
  if (with_prob(w, u[0])) {
    cos_theta = sample_draine_cos(u[0], g_d, a);
  } else {
    cos_theta = sample_henyey_greenstein(u[0], g_hg);
  }
  ps.f = psl::lerp(w, eval_henyey_greenstein(cos_theta, g_hg), eval_draine(cos_theta, g_d, a));
  ps.pdf = ps.f;

  auto sin_theta = psl::safe_sqrt(1 - sqr(cos_theta));
  auto phi = 2 * Pi * u[1];
  ps.wo = coordinate_system(wi) * spherical_to_cartesian(phi, sin_theta, cos_theta);

  return ps;
}

}  // namespace pine