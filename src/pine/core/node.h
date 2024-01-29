#pragma once

#include <pine/core/vecmath.h>
#include <pine/core/image.h>
#include <pine/core/log.h>

#include <psl/optional.h>
#include <psl/variant.h>
#include <psl/math.h>

namespace pine {

struct NodeEvalCtx {
  NodeEvalCtx(vec3 p, vec3 n, vec2 uv) : p{p}, n{n}, uv{uv} {};
  vec3 p;
  vec3 n;
  vec2 uv;
};

template <typename T>
struct Mnode;
using Nodef = Mnode<float>;
using Node3f = Mnode<vec3>;

template <typename T>
struct NodeConstant;
struct NodePosition;
struct NodeNormal;
struct NodeUV;
template <typename T, char op>
struct NodeBinary;
template <typename T, char op>
struct NodeUnary;
struct NodeCheckerboard;
struct NodeNoisef;
struct NodeNoise3f;
struct NodeImage;
struct NodeComponent;
struct NodeToVec3;

template <>
struct Mnode<float> {
  using Types = psl::TypePack<NodeConstant<float>, NodeBinary<float, '+'>, NodeBinary<float, '-'>,
                              NodeBinary<float, '*'>, NodeBinary<float, '/'>,
                              NodeBinary<float, '^'>, NodeUnary<float, '-'>, NodeUnary<float, 'a'>,
                              NodeUnary<float, 's'>, NodeUnary<float, 'r'>, NodeUnary<float, 'f'>,
                              NodeComponent, NodeNoisef, NodeCheckerboard>;

  template <typename T>
  requires psl::one_of<T, Types>
  Mnode(T value);
  Mnode(float);
  ~Mnode();

  Mnode(Mnode&&);
  Mnode(const Mnode&);
  Mnode& operator=(Mnode&&);
  Mnode& operator=(const Mnode&);

  float operator()(const NodeEvalCtx& nc) const;
  float eval(const NodeEvalCtx& ctx) const {
    return (*this)(ctx);
  }

private:
  psl::Box<psl::CopyTemplateArguments<psl::Variant, Types>> value;
};

template <>
struct Mnode<vec3> {
  using Types = psl::TypePack<Mnode<float>, NodePosition, NodeNormal, NodeUV, NodeConstant<vec3>,
                              NodeBinary<vec3, '+'>, NodeBinary<vec3, '-'>, NodeBinary<vec3, '*'>,
                              NodeBinary<vec3, '/'>, NodeBinary<vec3, '^'>, NodeUnary<vec3, '-'>,
                              NodeUnary<vec3, 'a'>, NodeUnary<vec3, 's'>, NodeUnary<vec3, 'r'>,
                              NodeUnary<vec3, 'f'>, NodeToVec3, NodeNoise3f, NodeImage>;

  template <typename T>
  requires psl::one_of<T, Types>
  Mnode(T value);
  Mnode(vec3);
  ~Mnode();

  Mnode(Mnode&&);
  Mnode(const Mnode&);
  Mnode& operator=(Mnode&&);
  Mnode& operator=(const Mnode&);

  vec3 operator()(const NodeEvalCtx& nc) const;
  vec3 eval(const NodeEvalCtx& ctx) const {
    return (*this)(ctx);
  }

private:
  psl::Box<psl::CopyTemplateArguments<psl::Variant, Types>> value;
};

template <typename T>
struct NodeConstant {
  NodeConstant(T value) : value{value} {};
  T eval(const NodeEvalCtx&) const {
    return value;
  }

private:
  T value;
};

struct NodePosition {
  vec3 eval(const NodeEvalCtx& ctx) const {
    return ctx.p;
  }
};
struct NodeNormal {
  vec3 eval(const NodeEvalCtx& ctx) const {
    return ctx.n;
  }
};
struct NodeUV {
  vec3 eval(const NodeEvalCtx& ctx) const {
    return vec3{ctx.uv};
  }
};

template <typename T, char op>
struct NodeBinary {
  NodeBinary(Mnode<T> a, Mnode<T> b) : a{psl::move(a)}, b(psl::move(b)){};
  T eval(const NodeEvalCtx& ctx) const {
    using psl::pow;
    if constexpr (op == '+')
      return a(ctx) + b(ctx);
    else if constexpr (op == '-')
      return a(ctx) - b(ctx);
    else if constexpr (op == '*')
      return a(ctx) * b(ctx);
    else if constexpr (op == '/')
      return a(ctx) / b(ctx);
    else if constexpr (op == '^')
      return pow(a(ctx), b(ctx));
    else
      static_assert(psl::deferred_bool<false, T>, "");
  }

private:
  Mnode<T> a, b;
};

template <typename T, char op>
struct NodeUnary {
  NodeUnary(Mnode<T> x) : x{psl::move(x)} {};
  T eval(const NodeEvalCtx& ctx) const {
    using psl::abs;
    using psl::fract;
    using psl::sqr;
    using psl::sqrt;
    if constexpr (op == '-')
      return -x(ctx);
    else if constexpr (op == 'a')
      return abs(x(ctx));
    else if constexpr (op == 's')
      return sqr(x(ctx));
    else if constexpr (op == 'r')
      return sqrt(x(ctx));
    else if constexpr (op == 'f')
      return fract(x(ctx));
    else
      static_assert(psl::deferred_bool<false, T>, "");
  }

private:
  Mnode<T> x;
};

struct NodeComponent {
  NodeComponent(Mnode<vec3> x, int n) : x{psl::move(x)}, n{n} {
    if (n < 0 || n > 2)
      exception("NodeComponent's second parameter should be 0, 1, or 2, but get ", n);
  };
  float eval(const NodeEvalCtx& ctx) const {
    CHECK_RANGE(n, 0, 2);
    return x(ctx)[n];
  }

private:
  Mnode<vec3> x;
  int n;
};

struct NodeToVec3 {
  NodeToVec3(Mnode<float> x) : x{psl::move(x)} {};
  NodeToVec3(Mnode<float> x, Mnode<float> y, Mnode<float> z)
      : x{psl::move(x)}, y{psl::move(y)}, z{psl::move(z)} {};
  vec3 eval(const NodeEvalCtx& ctx) const {
    if (!y)
      return vec3{x(ctx)};
    return vec3{x(ctx), (*y)(ctx), (*z)(ctx)};
  }

  Mnode<float> x;
  psl::optional<Mnode<float>> y, z;
};

struct NodeNoisef {
  NodeNoisef(Node3f p, Nodef frequency, Nodef octaves)
      : p{psl::move(p)}, frequency{psl::move(frequency)}, octaves{psl::move(octaves)} {};
  float eval(const NodeEvalCtx& ctx) const;

private:
  Node3f p;
  Nodef frequency;
  Nodef octaves;
};

struct NodeNoise3f {
  NodeNoise3f(Node3f p, Nodef frequency, Nodef octaves)
      : p{psl::move(p)}, frequency{psl::move(frequency)}, octaves{psl::move(octaves)} {};
  vec3 eval(const NodeEvalCtx& ctx) const;

private:
  Node3f p;
  Nodef frequency;
  Nodef octaves;
};

struct NodeCheckerboard {
  NodeCheckerboard(Node3f p, float ratio = 0.5f) : p{psl::move(p)}, ratio{ratio} {};
  float eval(const NodeEvalCtx& ctx) const;

private:
  Node3f p;
  float ratio;
};

struct NodeImage {
  NodeImage(Node3f p, psl::shared_ptr<Image> image) : p{psl::move(p)}, image{psl::move(image)} {};
  vec3 eval(const NodeEvalCtx& ctx) const;

private:
  Node3f p;
  psl::shared_ptr<Image> image;
};

template <typename T>
requires psl::one_of<T, Mnode<float>::Types>
Mnode<float>::Mnode(T value) : value{psl::move(value)} {};
inline Mnode<float>::Mnode(float value) : value{NodeConstant{value}} {};
inline Mnode<float>::~Mnode() = default;
inline Mnode<float>::Mnode(Mnode<float>&&) = default;
inline Mnode<float>::Mnode(const Mnode<float>&) = default;
inline Mnode<float>& Mnode<float>::operator=(Mnode<float>&&) = default;
inline Mnode<float>& Mnode<float>::operator=(const Mnode<float>&) = default;
inline float Mnode<float>::operator()(const NodeEvalCtx& nc) const {
  return value->dispatch([&nc](const auto& x) { return x.eval(nc); });
}

template <typename T>
requires psl::one_of<T, Mnode<vec3>::Types>
Mnode<vec3>::Mnode(T value) : value{psl::move(value)} {};
inline Mnode<vec3>::Mnode(vec3 value) : value{NodeConstant{value}} {};
inline Mnode<vec3>::~Mnode() = default;
inline Mnode<vec3>::Mnode(Mnode<vec3>&&) = default;
inline Mnode<vec3>::Mnode(const Mnode<vec3>&) = default;
inline Mnode<vec3>& Mnode<vec3>::operator=(Mnode<vec3>&&) = default;
inline Mnode<vec3>& Mnode<vec3>::operator=(const Mnode<vec3>&) = default;
inline vec3 Mnode<vec3>::operator()(const NodeEvalCtx& nc) const {
  return value->dispatch([&nc](const auto& x) { return vec3{x.eval(nc)}; });
}

void node_context(Context& ctx);

}  // namespace pine
