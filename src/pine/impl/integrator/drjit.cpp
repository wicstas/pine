#include <pine/impl/integrator/drjit.h>
#include <pine/core/parallel.h>
#include <pine/core/profiler.h>
#include <pine/core/fileio.h>
#include <psl/memory.h>

#include <drjit/struct.h>
#include <drjit/util.h>
#include <drjit/loop.h>
#include <drjit/jit.h>
#include <drjit/random.h>

namespace dr = drjit;

template <typename TV>
struct dr::struct_support<pine::Vector2<TV>> {
  static constexpr bool Defined = true;
  using type = struct_support;

  template <typename T, typename Func>
  static DRJIT_INLINE void apply_1(T &v, Func func) {
    func(v.x);
    func(v.y);
  }
  template <typename T1, typename T2, typename Func>
  static DRJIT_INLINE void apply_2(T1 &v1, T2 &v2, Func func) {
    func(v1.x, v2.x);
    func(v1.y, v2.y);
  }
  template <typename T1, typename T2, typename T3, typename Func>
  static DRJIT_INLINE void apply_3(T1 &v1, T2 &v2, T3 &v3, Func func) {
    func(v1.x, v2.x, v3.x);
    func(v1.y, v2.y, v3.y);
  }
  template <typename T, typename Func>
  static DRJIT_INLINE void apply_label(T &v, Func func) {
    func("x", v.x);
    func("y", v.y);
  }
};

template <typename TV>
struct dr::struct_support<pine::Vector3<TV>> {
  static constexpr bool Defined = true;
  using type = struct_support;

  template <typename T, typename Func>
  static DRJIT_INLINE void apply_1(T &v, Func func) {
    func(v.x);
    func(v.y);
    func(v.z);
  }
  template <typename T1, typename T2, typename Func>
  static DRJIT_INLINE void apply_2(T1 &v1, T2 &v2, Func func) {
    func(v1.x, v2.x);
    func(v1.y, v2.y);
    func(v1.z, v2.z);
  }
  template <typename T1, typename T2, typename T3, typename Func>
  static DRJIT_INLINE void apply_3(T1 &v1, T2 &v2, T3 &v3, Func func) {
    func(v1.x, v2.x, v3.x);
    func(v1.y, v2.y, v3.y);
    func(v1.z, v2.z, v3.z);
  }
  template <typename T, typename Func>
  static DRJIT_INLINE void apply_label(T &v, Func func) {
    func("x", v.x);
    func("y", v.y);
    func("z", v.z);
  }
};

template <typename TV>
struct dr::struct_support<pine::Vector4<TV>> {
  static constexpr bool Defined = true;
  using type = struct_support;

  template <typename T, typename Func>
  static DRJIT_INLINE void apply_1(T &v, Func func) {
    func(v.x);
    func(v.y);
    func(v.z);
    func(v.w);
  }
  template <typename T1, typename T2, typename Func>
  static DRJIT_INLINE void apply_2(T1 &v1, T2 &v2, Func func) {
    func(v1.x, v2.x);
    func(v1.y, v2.y);
    func(v1.z, v2.z);
    func(v1.w, v2.w);
  }
  template <typename T1, typename T2, typename T3, typename Func>
  static DRJIT_INLINE void apply_3(T1 &v1, T2 &v2, T3 &v3, Func func) {
    func(v1.x, v2.x, v3.x);
    func(v1.y, v2.y, v3.y);
    func(v1.z, v2.z, v3.z);
    func(v1.w, v2.w, v3.w);
  }
  template <typename T, typename Func>
  static DRJIT_INLINE void apply_label(T &v, Func func) {
    func("x", v.x);
    func("y", v.y);
    func("z", v.z);
    func("w", v.w);
  }
};

namespace pine {

using floatp = dr::LLVMArray<float>;
// using floatp = dr::DynamicArray<float>;
using uint32p = dr::uint32_array_t<floatp>;
using vec2p = pine::Vector2<floatp>;
using vec3p = pine::Vector3<floatp>;
using Mask = dr::mask_t<floatp>;

struct Ray_ {
  vec3p o;
  vec3p d;
  floatp tmin = 1e-4f;
  floatp tmax = float_max;

  vec3p operator()(const floatp &t) const {
    return o + t * d;
  }
};

static vec2p to_camera_space(vec2p p_film, vec2 fov2d) {
  p_film = (p_film - vec2p(0.5f)) * 2;
  return p_film * fov2d;
}

struct ThinLenCamera_ {
  Ray_ gen_ray(vec2p p_film) const {
    auto pc = to_camera_space(p_film, fov2d);
    auto p_focus = pine::normalize(vec3p(pc, floatp(1.0f)));
    auto rd = vec3(c2w.x) * p_focus.x + vec3(c2w.y) * p_focus.y + vec3(c2w.z) * p_focus.z;
    return Ray_(vec3(c2w * vec4(0, 0, 0, 1)), rd);
  }

  mat4 c2w;
  mat4 w2c;
  vec2 fov2d;
};

struct Interaction_ {
  vec3p p;
  vec3p n;
};

struct Shape_ {
  Mask intersect(Ray_ &r, Interaction_ &it) const {
    auto a = dot(r.d, r.d);
    auto b = 2 * dot(r.o - p, r.d);
    auto c = dot(r.o, r.o) + dot(p, p) - 2 * dot(r.o, vec3p(p)) - radius * radius;
    auto d = b * b - 4 * a * c;
    auto active = d > 0;
    d[d > 0] = sqrt(d);
    auto t = (-b - d) / (2 * a);
    t[t < r.tmin] = (-b + d) / (2 * a);
    active &= t > r.tmin & t < r.tmax;
    r.tmax[active] = t;
    it.n = select(active, pine::normalize(r(t) - p), it.n);
    it.p = select(active, p + it.n * radius, it.p);
    return active;
  }

  vec3 p;
  float radius;
};

struct Scene_ {
  psl::vector<Shape_> shapes;
};

struct RNG_ {
  RNG_(size_t size) : pcg(size) {
  }

  floatp get1d() {
    return pcg.next_float32();
  }
  vec2p get2d() {
    return {get1d(), get1d()};
  }
  vec3p get3d() {
    return {get1d(), get1d(), get1d()};
  }

private:
  dr::PCG32<floatp> pcg;
};

// static vec2p sample_disk_concentric(vec2p u) {
//   u = vec2p(u.x * 2 - 1.0f, u.y * 2 - 1.0f);
//   auto active = abs(u.x) > abs(u.y);
//   floatp theta, r;
//   r[active] = u.x;
//   theta[active] = Pi / 4.0f * u.y / u.x;
//   r[~active] = u.y;
//   theta[~active] = Pi / 2.0f - Pi / 4.0f * (u.x / u.y);
//   return r * vec2p(cos(theta), sin(theta));
// }

// static floatp max(floatp a, floatp b) {
//   a[a < b] = b;
//   return a;
// }

// static vec3p cosine_weighted_hemisphere(vec2p u) {
//   auto d = sample_disk_concentric(u);
//   auto z = sqrt(1.0f - d.x * d.x - d.y * d.y);
//   // auto z = sqrt(max(1.0f - d.x * d.x - d.y * d.y, floatp(0.0f)));
//   return vec3p(d.x, d.y, z);
// }

void pine::DrJitIntegrator::render(Scene &scene) {
  jit_init(JitBackend::LLVM);
  jit_set_log_level_stderr(LogLevel::Warn);
  Profiler _("Rendering");

  auto isize = scene.camera.film().size();
  auto xs = dr::linspace<floatp>(0.0f, 1.0f, isize.x);
  auto ys = dr::linspace<floatp>(0.0f, 1.0f, isize.y);
  auto [coords_x, coords_y] = dr::meshgrid(xs, ys);

  auto scene_ = Scene_();
  for (const auto &geo : scene.geometries) {
    if (geo->shape.is<Sphere>()) {
      auto &s = geo->shape.as<Sphere>();
      scene_.shapes.emplace_back(s.c, s.r);
    }
  }

  auto &cam = scene.camera.as<ThinLenCamera>();
  auto camera = ThinLenCamera_{cam.c2w, cam.w2c, cam.fov2d};
  auto sampler = RNG_(area(isize));
  auto ray = camera.gen_ray({coords_x, coords_y});

  auto L = vec3p(0);
  auto it = Interaction_();
  for (const auto &shape : scene_.shapes) {
    shape.intersect(ray, it);
  }

  for (int sp = 0; sp < samples_per_pixel; sp++) {
    auto shadowr = Ray_(it.p, pine::normalize(sampler.get3d() - vec3p(0.5f)));

    auto shadowit = Interaction_();
    auto hit = Mask(false);
    for (const auto &shape : scene_.shapes) {
      hit |= shape.intersect(shadowr, shadowit);
    }
    L += select(hit, vec3p(0), vec3p(1));
  }

  L /= samples_per_pixel;

  parallel_for(isize, [&](vec2i p) {
    auto i = p.x + p.y * isize.x;
    scene.camera.film().add_sample(p, vec3(L.x[i], L.y[i], L.z[i]));
  });
}

}  // namespace pine