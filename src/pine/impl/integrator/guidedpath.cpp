#include <pine/impl/integrator/guidedpath.h>
#include <pine/core/sampling.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

namespace pine {

namespace {
struct RadianceSample {
  RadianceSample() = default;
  RadianceSample(vec3 p, vec3 w, vec3 flux_color)
      : p{p}, w{w}, l{length(flux_color)}, flux_color{flux_color} {
  }
  vec3 p;
  vec3 w;
  float l;
  vec3 flux_color;
};

struct PgSample {
  vec3 w;
  float pdf = 0;
};

struct QuadTreeInputSample {
  vec2 sc;
  float flux = 0;
};

struct QuadTreeSample {
  vec2 sc;
  float pdf = 0;
};

struct QuadNode;
using QuadNodes = psl::vector<QuadNode>;
struct QuadNode {
  QuadNode() = default;
  QuadNode(vec2 lower, vec2 upper) : lower{lower}, upper{upper} {
  }

  float get_footprint(const QuadNodes& nodes, vec2 sc) const {
    if (is_leaf())
      return (upper[0] - lower[0]) / 2;
    else
      return child(nodes, sc).get_footprint(nodes, sc);
  }
  vec3 flux_estimate(const QuadNodes& nodes, vec2 sc) const {
    if (is_leaf())
      return vec3{flux_color[0], flux_color[1], flux_color[2]} / area(upper - lower);
    else
      return child(nodes, sc).flux_estimate(nodes, sc);
  }

  void add_sample(QuadNodes& nodes, vec2 sc, float footprint, float inv_sq_fp, float flux,
                  vec3 flux_color) {
    CHECK_GE(flux, 0);
    this->flux += flux;
    this->flux_color[0] += flux_color[0];
    this->flux_color[1] += flux_color[1];
    this->flux_color[2] += flux_color[2];
    if (!is_leaf())
      child(nodes, sc).add_sample(nodes, sc, footprint, inv_sq_fp, flux, flux_color);
    return;

    auto x0 = psl::max(lower[0], sc[0] - footprint);
    auto x1 = psl::min(upper[0], sc[0] + footprint);
    auto y0 = psl::max(lower[1], sc[1] - footprint);
    auto y1 = psl::min(upper[1], sc[1] + footprint);
    auto weight = psl::max(x1 - x0, 0.0f) * psl::max(y1 - y0, 0.0f) * inv_sq_fp;
    this->flux += weight * flux;
    this->flux_color[0] += weight * flux_color[0];
    this->flux_color[1] += weight * flux_color[1];
    this->flux_color[2] += weight * flux_color[2];
    if (psl::abs(weight) < 0.01f)
      return;
    if (!is_leaf()) {
      for (int i = 0; i < 4; i++)
        child(nodes, i).add_sample(nodes, sc, footprint, inv_sq_fp, flux, flux_color);
    }
  }

  void refine(QuadNodes& nodes, float total_flux) {
    if (flux > total_flux * 0.01f && depth() < 15) {
      if (is_leaf()) {
        children_offset = static_cast<int>(nodes.size());
        auto mid = (lower + upper) / 2;
        nodes.emplace_back(vec2{lower[0], lower[1]}, vec2{mid[0], mid[1]});
        nodes.emplace_back(vec2{mid[0], lower[1]}, vec2{upper[0], mid[1]});
        nodes.emplace_back(vec2{lower[0], mid[1]}, vec2{mid[0], upper[1]});
        nodes.emplace_back(vec2{mid[0], mid[1]}, vec2{upper[0], upper[1]});
        for (int i = 0; i < 4; i++) {
          auto c = psl::move(child(nodes, i));
          c.flux = flux / 4;
          c.flux_color[0] = flux_color[0] / 4;
          c.flux_color[1] = flux_color[1] / 4;
          c.flux_color[2] = flux_color[2] / 4;
          c.refine(nodes, total_flux);
          child(nodes, i) = psl::move(c);
        }
      } else {
        for (int i = 0; i < 4; i++) {
          auto c = psl::move(child(nodes, i));
          c.refine(nodes, total_flux);
          child(nodes, i) = psl::move(c);
        }
      }
    } else {
      if (!is_leaf()) {
        children_offset = -1;
      }
    }
  }

  psl::optional<QuadTreeSample> sample(const QuadNodes& nodes, vec2d u, float pdf = 1) const {
    if (flux == 0.0f)
      return psl::nullopt;
    if (is_leaf()) {
      return QuadTreeSample{lerp(vec2{u}, lower, upper), pdf / (4 * Pi)};
    } else {
      auto rx = (child(nodes, 0).flux + child(nodes, 2).flux) / flux;
      if (u[0] < rx) {
        u[0] = u[0] / rx;
        auto ry = child(nodes, 0).flux / (child(nodes, 0).flux + child(nodes, 2).flux);
        if (u[1] < ry) {
          u[1] = u[1] / ry;
          return child(nodes, 0).sample(nodes, u, pdf * 4 * rx * ry);
        } else {
          u[1] = (u[1] - ry) / psl::max(1 - ry, epsilon);
          return child(nodes, 2).sample(nodes, u, pdf * 4 * rx * (1 - ry));
        }
      } else {
        u[0] = (u[0] - rx) / psl::max(1 - rx, epsilon);
        auto ry = child(nodes, 1).flux / (child(nodes, 1).flux + child(nodes, 3).flux);
        if (u[1] < ry) {
          u[1] = u[1] / ry;
          return child(nodes, 1).sample(nodes, u, pdf * 4 * (1 - rx) * ry);
        } else {
          u[1] = (u[1] - ry) / psl::max(1 - ry, epsilon);
          return child(nodes, 3).sample(nodes, u, pdf * 4 * (1 - rx) * (1 - ry));
        }
      }
    }
  }
  float pdf(const QuadNodes& nodes, vec2 sc) const {
    if (is_leaf()) {
      return 1.0f / (4 * Pi);
    } else {
      if (flux == 0.0f)
        return 0.0f;
      auto& c = child(nodes, sc);
      return 4 * c.flux / flux * c.pdf(nodes, sc);
    }
  }
  float flux_density(const QuadNodes& nodes, vec2 sc) const {
    if (flux == 0)
      return flux;
    if (is_leaf())
      return flux / (4 * Pi * area(upper - lower));
    else
      return child(nodes, sc).flux_density(nodes, sc);
  }
  bool is_leaf() const {
    return children_offset == -1;
  }
  int depth() const {
    return -psl::log2(upper[0] - lower[0]);
  }
  int tree_depth(const QuadNodes& nodes, vec2 sc) const {
    if (is_leaf())
      return 1;
    else
      return 1 + child(nodes, sc).tree_depth(nodes, sc);
  }
  QuadNode& child(QuadNodes& nodes, vec2 sc) const {
    auto index = 0;
    index += int{sc[0] > (lower[0] + upper[0]) / 2};
    index += 2 * int{sc[1] > (lower[1] + upper[1]) / 2};
    return child(nodes, index);
  }
  const QuadNode& child(const QuadNodes& nodes, vec2 sc) const {
    auto index = 0;
    index += int{sc[0] > (lower[0] + upper[0]) / 2};
    index += 2 * int{sc[1] > (lower[1] + upper[1]) / 2};
    return child(nodes, index);
  }
  QuadNode& child(QuadNodes& nodes, int index) const {
    return nodes[children_offset + index];
  }
  const QuadNode& child(const QuadNodes& nodes, int index) const {
    return nodes[children_offset + index];
  }

  Atomic<float> flux{0};
  Atomic<float> flux_color[3]{0, 0, 0};
  vec2 lower;
  vec2 upper;
  int children_offset = -1;
};

struct QuadTree {
  QuadTree() {
    nodes.push_back(QuadNode{vec2{0, 0}, vec2{1, 1}});
  }
  float get_footprint(vec2 sc) const {
    return root().get_footprint(nodes, sc);
  }
  vec3 flux_estimate(vec2 sc) const {
    if (n_samples == 0)
      return vec3{0.0f};
    return root().flux_estimate(nodes, sc) / int{n_samples};
  }

  void add_sample(vec2 sc, float flux, vec3 flux_color) {
    CHECK_GE(flux, 0);
    auto fp = get_footprint(sc);
    root().add_sample(nodes, sc, fp, 1.0f / psl::sqr(fp), flux, flux_color);
    n_samples += 1;
  }

  void refine() {
    auto c = psl::move(root());
    c.refine(nodes, root().flux);
    root() = psl::move(c);
  }

  psl::optional<QuadTreeSample> sample(vec2d u) const {
    return root().sample(nodes, u);
  }
  float pdf(vec2 sc) const {
    return root().pdf(nodes, sc);
  }
  void clear() {
    for (auto& node : nodes) {
      node.flux = 0;
      node.flux_color[0] = 0;
      node.flux_color[1] = 0;
      node.flux_color[2] = 0;
    }
    n_samples = 0;
  }
  int tree_depth(vec2 sc) const {
    return root().tree_depth(nodes, sc);
  }

  QuadNode& root() {
    return nodes[0];
  }
  const QuadNode& root() const {
    return nodes[0];
  }

  QuadNodes nodes;
  Atomic<int> n_samples{0};
};

struct SpatialNode;
using SpatialNodes = psl::vector<SpatialNode>;

struct SpatialNode {
  SpatialNode() = default;
  SpatialNode(AABB aabb, QuadTree quad)
      : aabb{aabb}, axis{max_axis(aabb.diagonal())}, guide{psl::move(quad)}, collector{guide} {
  }

  float get_footprint() {
    return aabb.diagonal()[axis] / 2;
  }
  SpatialNode& add_sample(RadianceSample s) {
    CHECK_GE(s.l, 0);
    n_samples += 1;
    collector->add_sample(inverse_uniform_sphere(s.w), s.l, s.flux_color);
    return *this;
  }
  SpatialNode& refine(SpatialNodes& nodes, int k) {
    if (is_leaf()) {
      const auto c = 2000;
      const auto threshold = static_cast<size_t>(c * psl::sqrt(psl::sqrt<float>(1 << k)));
      if (n_samples > threshold) {
        split_p = psl::lerp(0.5f, aabb.lower[axis], aabb.upper[axis]);
        auto [l_aabb, r_aabb] = aabb.split_half(axis);
        children_indices[0] = static_cast<int>(nodes.size()) + 0;
        children_indices[1] = static_cast<int>(nodes.size()) + 1;
        nodes.emplace_back(l_aabb, *collector);
        nodes.emplace_back(r_aabb, psl::move(*collector));
        guide = collector = psl::nullopt;
        auto c0 = psl::move(child(nodes, 0));
        auto c1 = psl::move(child(nodes, 1));
        c0.n_samples = n_samples / 2;
        c1.n_samples = n_samples / 2;
        c0.refine(nodes, k);
        c1.refine(nodes, k);
        child(nodes, 0) = psl::move(c0);
        child(nodes, 1) = psl::move(c1);
      } else {
        Profiler _("Quad refine");
        collector->refine();
        guide = collector;
        collector->clear();
      }
      n_samples = 0;
    } else {
      auto c0 = psl::move(nodes[children_indices[0]]);
      auto c1 = psl::move(nodes[children_indices[1]]);
      c0.refine(nodes, k);
      c1.refine(nodes, k);
      nodes[children_indices[0]] = psl::move(c0);
      nodes[children_indices[1]] = psl::move(c1);
    }
    return *this;
  }

  psl::optional<PgSample> sample(vec2 u) const {
    CHECK(is_leaf());
    if (auto qs = guide->sample(u)) {
      return PgSample{uniform_sphere(qs->sc), qs->pdf};
    } else
      return psl::nullopt;
  }
  float pdf(vec3 w) const {
    CHECK(is_leaf());
    auto sc = inverse_uniform_sphere(w);
    return guide->pdf(sc);
  }

  SpatialNode& traverse(SpatialNodes& nodes, vec3 p) {
    if (is_leaf())
      return *this;
    else
      return child(nodes, p).traverse(nodes, p);
  }
  const SpatialNode& traverse(const SpatialNodes& nodes, vec3 p) const {
    if (is_leaf())
      return *this;
    else
      return child(nodes, p).traverse(nodes, p);
  }

  bool is_leaf() const {
    return children_indices[0] == -1;
    if (children_indices[0] != -1) {
      CHECK(children_indices[1] != -1);
      CHECK(!guide);
      CHECK(!collector);
      return false;
    } else {
      CHECK(children_indices[1] == -1);
      CHECK(guide);
      CHECK(collector);
      return true;
    }
  }

  SpatialNode& child(SpatialNodes& nodes, int index) {
    CHECK_RANGE(index, 0, 1);
    return nodes[children_indices[index]];
  }
  const SpatialNode& child(const SpatialNodes& nodes, int index) const {
    CHECK_RANGE(index, 0, 1);
    return nodes[children_indices[index]];
  }
  SpatialNode& child(SpatialNodes& nodes, vec3 p) {
    auto index = int{p[axis] >= split_p};
    return child(nodes, index);
  }
  const SpatialNode& child(const SpatialNodes& nodes, vec3 p) const {
    auto index = int{p[axis] >= split_p};
    return child(nodes, index);
  }

  AABB aabb;
  int axis;
  float split_p;
  Atomic<size_t> n_samples{0};
  psl::optional<QuadTree> guide;
  psl::optional<QuadTree> collector;
  int children_indices[2]{-1, -1};
};

struct SpatialTree {
  SpatialTree() = default;
  SpatialTree(AABB top_aabb, QuadTree quad_tree) {
    nodes.push_back(SpatialNode{top_aabb, quad_tree});
  }
  void add_sample(SpatialNode& leaf, RadianceSample s, vec3 up) {
    CHECK_GE(s.l, 0.0f);
    up -= vec3{0.5f};
    s.p += leaf.get_footprint() * 2 * up;
    auto aabb = root().aabb;
    for (int i = 0; i < 3; i++) {
      if (s.p[i] < aabb.lower[i])
        s.p[i] += 2 * (aabb.lower[i] - s.p[i]);
      else if (s.p[i] > aabb.upper[i])
        s.p[i] += 2 * (aabb.upper[i] - s.p[i]);
    }
    traverse(s.p).add_sample(s);
  }
  void refine(int k) {
    root() = SpatialNode{root()}.refine(nodes, k);
  }
  SpatialNode& traverse(vec3 p) {
    return root().traverse(nodes, p);
  }
  SpatialNode& root() {
    return nodes[0];
  }
  const SpatialNode& root() const {
    return nodes[0];
  }

private:
  SpatialNodes nodes;
};

}  // namespace

static SpatialTree sd_tree;

static bool use_guide = false;
static bool collect = true;

void GuidedPathIntegrator::render(Scene& scene) {
  for (const auto& geometry : scene.geometries)
    if (geometry->shape.is<Plane>())
      Fatal("GuidedPathIntegrator doesn't support `Plane`, please use `Rect` or `Disk` instead");

  accel.build(&scene);
  light_sampler.build(&scene);
  auto& film = scene.camera.film();
  film.clear();

  Profiler _("Rendering");

  auto quad_tree = QuadTree{};
  quad_tree.root().flux = 1;
  quad_tree.refine();
  sd_tree = SpatialTree{scene.get_aabb(), quad_tree};
  use_guide = false;
  collect = true;

  auto total_pixels = static_cast<size_t>(area(film.size()));
  auto total_samples = total_pixels * samples_per_pixel;
  auto total_samples_all = total_samples + total_pixels * estimate_samples;
  auto initial_samples = size_t{1024 * 16};
  auto n_iterations =
      static_cast<int>(psl::ceil(psl::log2(total_samples / initial_samples + 1.0f)));
  Debug("[GuidedPath]", n_iterations, " learning iterations");
  sd_tree.root().n_samples = initial_samples * 4;
  sd_tree.refine(0);

  auto current_sample_index = 0;
  auto current_samples = size_t{0};
  auto current_film = film;

  set_progress(0);
  for (int iteration = 0; iteration < n_iterations; iteration++) {
    auto iter_samples = initial_samples * (1 << iteration);

    if (iteration + 1 == n_iterations) {
      if (!use_estimate)
        collect = false;
    }

    {
      Profiler _("Collecting");
      auto downsize = psl::min(psl::sqrt(static_cast<double>(iter_samples) / total_pixels), 1.0);
      auto iter_size = vec2i{film.size() * downsize};
      auto iter_n_pass = iter_samples / area(iter_size);
      for (size_t si = 0; si < iter_n_pass; si++) {
        parallel_for(iter_size, [&](vec2i p) {
          auto& sampler = samplers[threadIdx];
          sampler.start_pixel(p, current_sample_index);

          auto p_film = vec2(p + sampler.get2d()) / iter_size;
          auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
          auto L = radiance(scene, ray, sampler, 0, 1.0f, vec3{0.0f}, true, false);
          // CHECK(!L.has_inf());
          // CHECK(!L.has_nan());
          L = clamp(L, vec3{0.0f}, vec3{20.0f});
          if (!use_estimate)
            current_film.add_sample(p_film * current_film.size(), L);
        });
        current_sample_index += 1;
        current_samples += area(iter_size);
        set_progress(static_cast<double>(current_samples) / total_samples_all);
      }

      if (!use_estimate) {
        if (iteration == 0) {
          film = current_film;
        } else {
          film = combine(film, current_film, 1, 2);
        }
        current_film.clear();
      }
    }
    {
      Profiler _("Refinement");
      if (collect)
        sd_tree.refine(iteration);
      use_guide = true;
    }
  }

  if (use_estimate) {
    film.clear();
    for (int si = 0; si < estimate_samples; si++) {
      parallel_for(film.size(), [&](vec2i p) {
        auto& sampler = samplers[threadIdx];
        sampler.start_pixel(p, current_sample_index);
        auto p_film = vec2(p + sampler.get2d()) / film.size();
        // film.add_sample(p,
        //                 color_map(sd_tree.traverse({-1, 1, 2}).guide->tree_depth(p_film)
        //                 / 10.0f));
        // return;
        auto ray = scene.camera.gen_ray(p_film, sampler.get2d());
        auto L = radiance_estimate(scene, ray, sampler, 0);
        CHECK(!L.has_inf());
        CHECK(!L.has_nan());
        L = clamp(L, vec3{0.0f}, vec3{20.0f});
        film.add_sample(p, L);
      });
      current_sample_index += 1;
      current_samples += area(film.size());
      set_progress(static_cast<double>(current_samples) / total_samples_all);
    }
  }

  set_progress(1.0f);
}

vec3 GuidedPathIntegrator::radiance_estimate(Scene& scene, Ray ray, Sampler& sampler, int depth) {
  auto L = vec3{0.0f};

  auto it = Interaction{};
  if (!intersect(ray, it)) {
    if (scene.env_light)
      L += scene.env_light->color(ray.d);
    return L;
  }

  if (it.material()->is<EmissiveMaterial>()) {
    L += it.material()->le({it, -ray.d});
    return L;
  }

  if (depth + 1 == max_depth)
    return vec3{0.0f};

  if (dot(it.n, -ray.d) < 0)
    it.n = -it.n;

  if (it.material()->is_delta()) {
    auto msc = MaterialSampleCtx{it, -ray.d, sampler.get1d(), sampler.get2d()};
    if (auto bs = it.material()->sample(msc)) {
      auto li = radiance_estimate(scene, it.spawn_ray(bs->wo), sampler, depth + 1);
      auto cosine = absdot(it.n, bs->wo);
      return li * cosine * bs->f / bs->pdf;
    } else {
      return vec3{0.0f};
    }
  }

  auto direct_light = [&](LightSample ls) {
    if (!hit(it.spawn_ray(ls.wo, ls.distance))) {
      auto mec = MaterialEvalCtx(it, -ray.d, ls.wo);
      auto f = it.material()->f(mec);
      return ls.le / ls.pdf * psl::max(dot(it.n, ls.wo), 0.0f) * f;
    } else {
      return vec3{0.0f};
    }
  };

  auto footprint = sd_tree.traverse(it.p).get_footprint();
  auto p = it.p + footprint * (sampler.get3d() - vec3{0.5f}) * 2;
  auto aabb = sd_tree.root().aabb;
  for (int i = 0; i < 3; i++) {
    if (p[i] < aabb.lower[i])
      p[i] += 2 * (aabb.lower[i] - p[i]);
    else if (p[i] > aabb.upper[i])
      p[i] += 2 * (aabb.upper[i] - p[i]);
  }
  auto& quad = *sd_tree.traverse(p).guide;

  auto guide_select_prob = 0.8f;
  if (sampler.get1d() < guide_select_prob) {
    if (auto qs = quad.sample(sampler.get2d())) {
      auto wo = uniform_sphere(qs->sc);
      auto mec = MaterialEvalCtx{it, -ray.d, wo};
      auto le = quad.flux_estimate(qs->sc) / (4);
      auto mis_term = balance_heuristic(1, qs->pdf, 1, it.material()->pdf(mec)) / guide_select_prob;
      L += le * it.material()->f(mec) / qs->pdf * mis_term;
    }
  } else {
    if (auto bs = it.material()->sample({it, -ray.d, sampler.get1d(), sampler.get2d()})) {
      auto sc = inverse_uniform_sphere(bs->wo);
      auto le = quad.flux_estimate(sc) / (4);
      auto mis_term = balance_heuristic(1, bs->pdf, 1, quad.pdf(sc)) / (1 - guide_select_prob);
      L += le * bs->f / bs->pdf * mis_term;
    }
  }
  if (scene.env_light)
    if (auto ls = scene.env_light->sample(it.n, sampler.get2d()))
      L += direct_light(*ls);

  return L;
}

vec3 GuidedPathIntegrator::radiance(Scene& scene, Ray ray, Sampler& sampler, int depth,
                                    float prev_sample_pdf, vec3 prev_n, bool prev_delta,
                                    bool mis_env_light) {
  auto wi = -ray.d;
  auto it = Interaction{};
  if (!intersect(ray, it)) {
    if (scene.env_light) {
      if (prev_delta) {
        return scene.env_light->color(ray.d);
      } else if (mis_env_light) {
        auto le = scene.env_light->color(ray.d);
        auto light_pdf = scene.env_light->pdf(prev_n, ray.d);
        auto mis_term = power_heuristic(1, prev_sample_pdf, 1, light_pdf);
        // mis_term shouldn't be added to sd_tree?
        return le * mis_term;
      }
    }
    return vec3{0.0f};
  }

  if (it.material()->is<EmissiveMaterial>()) {
    if (prev_delta) {
      return it.material()->le({it, -wi});
    } else if (mis_env_light) {
      auto le = it.material()->le({it, -wi});
      auto light_pdf = light_sampler.pdf(it.geometry, it, ray, prev_n);
      auto mis_term = power_heuristic(1, prev_sample_pdf, 1, light_pdf);
      return le * mis_term;
    }
    return vec3{0.0f};
  }

  if (depth + 1 == max_depth)
    return vec3{0.0f};

  auto lo = vec3{0.0f};

  SpatialNode& leaf = sd_tree.traverse(it.p);
  auto direct_light = [&](LightSample ls, float pdf_g, bool collect_ = true) {
    if (!hit(it.spawn_ray(ls.wo, ls.distance))) {
      auto cosine = absdot(ls.wo, it.n);
      auto mec = MaterialEvalCtx(it, -ray.d, ls.wo);
      auto f = it.material()->f(mec);
      auto mis_term = power_heuristic(ls.pdf, pdf_g);
      // TODO: divides by pdf?
      if (collect && collect_)
        sd_tree.add_sample(leaf, {it.p, ls.wo, ls.le * cosine / ls.pdf}, sampler.get3d());
      return ls.le / ls.pdf * cosine * f * mis_term;
    } else {
      return vec3{0.0f};
    }
  };

  auto guide_select_prob = use_guide ? 0.75f : 0.0f;
  if (it.material()->is_delta())
    guide_select_prob = 0.0f;
  if (sampler.get1d() < guide_select_prob) {
    if (auto ps = leaf.sample(sampler.get2d())) {
      auto cosine = absdot(it.n, ps->w);
      auto mec = MaterialEvalCtx{it, wi, ps->w};
      auto f = it.material()->f(mec);
      if (psl::abs(ps->pdf) > 1e-7f && cosine > 1e-7f && length_squared(f) > 1e-10f) {
        auto li =
            radiance(scene, it.spawn_ray(ps->w), sampler, depth + 1, 0.0f, it.n, false, false);
        if (collect)
          sd_tree.add_sample(leaf, {it.p, ps->w, li * cosine / ps->pdf}, sampler.get3d());
        auto mis_term = power_heuristic(1, ps->pdf, 1, it.material()->pdf(mec)) / guide_select_prob;
        lo += li * cosine * f / ps->pdf * mis_term;
        CHECK(!(li * cosine * f / ps->pdf * mis_term).has_nan());
        if ((li * cosine * f / ps->pdf * mis_term).has_nan())
          Logs(li, cosine, f, ps->pdf, mis_term);
      }
    }

    if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
      lo += direct_light(*ls, 0.0f, false);
    }
    if (scene.env_light) {
      if (auto ls = scene.env_light->sample(it.n, sampler.get2d()))
        lo += direct_light(*ls, 0.0f, false);
    }
  } else {
    if (auto bs = it.material()->sample({it, wi, sampler.get1d(), sampler.get2d()})) {
      auto cosine = absdot(it.n, bs->wo);
      if (psl::abs(bs->pdf) > 1e-7f && cosine > 1e-7f && length_squared(bs->f) > 1e-10f) {
        auto li = radiance(scene, it.spawn_ray(bs->wo), sampler, depth + 1, bs->pdf, it.n,
                           it.material()->is_delta(), true);
        if (collect)
          sd_tree.add_sample(leaf, {it.p, bs->wo, li * cosine / bs->pdf}, sampler.get3d());
        auto mis_term =
            guide_select_prob == 0.0f
                ? 1.0f
                : power_heuristic(1, bs->pdf, 1, leaf.pdf(bs->wo)) / (1 - guide_select_prob);
        lo += li * cosine * bs->f / bs->pdf * mis_term;
        CHECK(!(li * cosine * bs->f / bs->pdf * mis_term).has_nan());
        if ((li * cosine * bs->f / bs->pdf * mis_term).has_nan())
          Logs(li, cosine, bs->f, bs->pdf, mis_term);
      }
    }

    if (!it.material()->is_delta())
      if (auto ls = light_sampler.sample(it.p, it.n, sampler.get1d(), sampler.get2d())) {
        lo += direct_light(*ls, ls->light->is_delta() ? 0.0f : it.material()->pdf({it, wi, ls->wo}),
                           false);
      }
    if (!it.material()->is_delta() && scene.env_light) {
      if (auto ls = scene.env_light->sample(it.n, sampler.get2d()))
        lo += direct_light(*ls, it.material()->pdf({it, wi, ls->wo}), false);
    }
  }

  return lo;
}

}  // namespace pine