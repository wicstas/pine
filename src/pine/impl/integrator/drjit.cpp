#include <pine/impl/integrator/drjit.h>
#include <pine/core/fileio.h>
#include <psl/memory.h>

#include <drjit/dynamic.h>
#include <drjit/struct.h>
#include <drjit/vcall.h>
#include <drjit/util.h>
#include <drjit/jit.h>

#define PINE_NAMESPACE_BEGIN namespace pine {
#define PINE_NAMESPACE_END }  // namespace pine

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
using uint32p = dr::uint32_array_t<floatp>;
using vec2p = pine::Vector2<floatp>;
using vec3p = pine::Vector3<floatp>;
using Mask = dr::mask_t<floatp>;

struct Ray_ {
  vec3p o;
  vec3p d;

  vec3p operator()(const floatp &t) const {
    return o + t * d;
  }
};

Ray_ make_rays(const vec2p &p) {
  Ray_ ray;
  ray.o = vec3p(p.x, p.y, 1.f);
  ray.d = vec3p(0.f, 0.f, -1.f);
  return ray;
}

struct Shape_ {
  Shape_() {
  }
  ~Shape_() = default;

  virtual floatp intersect(const Ray_ &r, Mask = true) const = 0;

  DRJIT_VCALL_REGISTER(floatp, Base)

protected:
  floatp p;
};
using Shape_Ptr = dr::LLVMArray<Shape_ *>;

struct Sphere_ : Shape_ {
  Sphere_(vec3 o, float radius) {
  }
  floatp intersect(const Ray_ &r, Mask = true) const override {
    return r.o.x;
    // auto a = dot(r.d, r.d);
    // auto b = 2 * dot(r.o - vec3p(0, 0, 0), r.d);
    // auto c = dot(r.o, r.o) + dot(vec3p(0, 0, 0shapes[0]->intersect(rays)), vec3p(0, 0, 0)) - 2 *
    // dot(r.o, vec3p(0, 0, 0)) -
    //          floatp(rad * rad);
    // auto d = b * b - 4 * a * c;
    // d[d > 0] = sqrt(d);
    // auto t = (-b - d) / (2 * a);
    // t = dr::select(t < 0, (-b + d) / (2 * a), t);
    // return r(t);
    // return d >= 0 && t > 0;
  }

  // vec3 o;
  // float rad;
};

PINE_NAMESPACE_END
DRJIT_VCALL_BEGIN(pine::Shape_)
DRJIT_VCALL_METHOD(intersect)
DRJIT_VCALL_END(pine::Shape_)
PINE_NAMESPACE_BEGIN

struct Scene_ {
  vec3p intersect_rays(const Ray_ &rays) const {
    return vec3p(shapes[0]->intersect(rays));
    // return *(vec3p *)(&c);
    // return select(mask, rays.o, vec3p(0));
  }

  psl::vector<Shape_Ptr> shapes;
};

void pine::DrJitIntegrator::render(Scene &) {
  jit_init(JitBackend::LLVM);

  jit_set_log_level_stderr(LogLevel::Debug);
  jit_set_flag(JitFlag::VCallRecord, true);
  jit_set_flag(JitFlag::VCallOptimize, true);

  auto scene = Scene_();
  scene.shapes.push_back(new Sphere_(vec3(0, 0, 0), 0.5f));

  auto width = 1024;
  auto idx = dr::linspace<floatp>(-1.f, 1.f, width);
  auto [coords_x, coords_y] = dr::meshgrid(idx, idx);
  auto rays = make_rays({coords_x, coords_y});
  auto ps = scene.intersect_rays(rays);

  auto image = Array2d3f({width, width});
  auto index = 0u;
  for (auto &pixel : image) {
    pixel.x = ps.x[index];
    pixel.y = ps.y[index];
    pixel.z = ps.z[index];
    ++index;
  }
  save_image("images/image.png", image);
}

}  // namespace pine