#include <pine/impl/integrator/micro_render.h>
#include <pine/core/parallel.h>
#include <pine/core/profiler.h>
#include <pine/core/opengl.h>
#include <pine/core/scene.h>

namespace pine {

auto pl = vec3(0, 1.9f, 0.5f);
auto cl = vec3(1, 1, 1);

void push_pop(Texture<vec4>& texture) {
  CHECK(psl::is_power_of_2(texture.width()));
  CHECK(texture.width() == texture.height());
  auto mipmaps = Array2d4f(texture.size);
  const auto n_min = texture.width() / 4;

  auto x_offset = [w = texture.width() / 2](int n) {
    auto sum = 0;
    for (int x = w; x > n; x /= 2) sum += x;
    return sum;
  };
  auto falloff = 1;
  for (int n = texture.width() / 2; n >= n_min; n /= 2) {
    auto& T = n == texture.width() / 2 ? texture.texels : mipmaps;
    for_2d({n, n}, [&](vec2i p) {
      auto P = vec2i(x_offset(n * 2) + p.x * 2, p.y * 2);
      mipmaps[{x_offset(n) + p.x, p.y}] =
          psl::max(T[P], T[P + vec2i(1, 0)], T[P + vec2i(0, 1)], T[P + vec2i(1, 1)]) / falloff;
    });
    falloff *= 2;
  }

  for (int n = n_min * 2; n <= texture.width(); n *= 2) {
    auto& T = n == texture.width() ? texture.texels : mipmaps;
    for_2d({n, n}, [&](vec2i p) {
      auto P = vec2i(x_offset(n) + p.x, p.y);
      auto sample_p = vec2i(x_offset(n / 2) + p.x / 2, p.y / 2);
      if (vec3(T[P]).is_black()) T[P] = mipmaps[sample_p];
    });
  }
}

template <typename T>
T geometric_sum(T a, T n, T k) {
  return a * (psl::pow(n, k + 1) - 1) / (n - 1);
}

struct Disc {
  vec3 p, n, cd;
  float r;
};

float solid_angle(vec3 p, Disc d) {
  auto r = d.r / length(p - d.p);
  return 4 * Pi * r * r / (1 + r * r);
}

struct BSphere {
  BSphere(AABB aabb) {
    center = aabb.centroid();
    r = length(aabb.upper - aabb.lower) / 2;
  }
  vec3 center;
  float r;
};

psl::optional<vec2> project(vec3 sample_p, vec3 p_, vec3 n) {
  auto w2n = inverse(coordinate_system(n));
  auto p = normalize(w2n * (sample_p - p_));
  if (p.z <= 0) return {};
  auto phi = phi2pi(p.x, p.y) / Pi / 2;
  auto theta = 1 - psl::sqrt(p.x * p.x + p.y * p.y);
  if (!inside({phi, theta}, {0, 0}, vec2(1, 1))) return {};
  return vec2{phi, theta};
}

void MicroRenderIntegrator::render(Scene& scene) {
  if (scene.geometries.size() == 0) return;
  auto& camera = scene.camera;
  auto& film = scene.camera.film();
  auto accel = EmbreeAccel(&scene);

  auto cdf = Distribution1D(transform_vector(scene.geometries, [](auto& x) { return x->area(); }));
  auto radius = psl::sqrt(cdf.density_sum() / point_count) / Pi;

  auto window = GLWindow(film.size() * vec2i(3, 1), "MicroRenderGI");
  auto program = GLProgram("shaders/basic.vert", "shaders/mrgi.frag");
  auto vao = VAO(psl::vector_of<float>(-1, -1, -1, 1, 1, 1, 1, -1), 0, 2);

  program.set_uniform("film_size", film.size());
  auto position = Texture<vec3>(film.size(), 0);
  auto normal = Texture<vec3>(film.size(), 1);
  auto color = Texture<vec3>(film.size(), 2);
  auto direct = Texture<vec3>(film.size(), 3);
  auto debug = Texture<vec4>(vec2i(32, 32), 4);

  auto timer = Timer();
  auto discs = psl::vector<Disc>(point_count * 2);
  parallel_for(point_count, [&](int i) {
    auto& geo = scene.geometries[cdf.sample(radical_inverse(0, i)).i];
    auto gs = geo->sample({radical_inverse(1, i), radical_inverse(2, i)}, radical_inverse(3, i));
    auto cd = vec3(0);
    if (!accel.hit(spawn_ray1(gs.p, gs.n, pl - gs.p)))
      cd = cl * geo->material->albedo({gs.p, gs.n, gs.uv, gs.n}) *
           absdot(gs.n, normalize(pl - gs.p)) / (0.5f + distance_squared(gs.p, pl));
    discs[i + point_count] = {gs.p, gs.n, cd, radius};
  });

  /*
      0
    1   2
  3  4 5 6
  */

  auto child_index_offset = [](int i) {
    return psl::roundup2(i + 2) - 1 + (i + 1 - psl::roundup2(i + 2) / 2) * 2;
  };
  auto disc_add_index = 0u;
  auto add_disc = [&](Disc disc) {
    CHECK_LT(disc_add_index, discs.size());
    discs[disc_add_index++] = disc;
  };
  auto build = [&](Disc* pstart, Disc* pend, auto build) {
    if (pstart + 2 >= pend) return;
    auto range = psl::range(pstart, pend);
    auto aabb = AABB();
    for (const auto& disc : range) aabb.extend(disc.p);
    auto axis = max_axis(aabb.diagonal());
    psl::sort(range, [axis](const Disc& a, const Disc& b) { return a.p[axis] < b.p[axis]; });
    Disc a, b;
    AABB A, B;

    for (size_t i = 0; i < range.size() / 2; i++) {
      A.extend(pstart[i].p);
      B.extend(pstart[i + range.size() / 2].p);
      a.cd += pstart[i].cd;
      b.cd += pstart[i + range.size() / 2].cd;
    }
    a.cd /= range.size() / 2;
    a.p = BSphere(A).center;
    a.r = BSphere(A).r;
    b.p = BSphere(B).center;
    b.r = BSphere(B).r;
    b.cd /= range.size() / 2;
    LOG("axis: ", axis, " count: ", range.size(), ' ', a.p, ' ', a.r, ' ', b.p, ' ', b.r);
    add_disc(a);
    add_disc(b);
    build(pstart, pstart + range.size() / 2, build);
    build(pstart + range.size() / 2, pend, build);
  };
  build(&discs[point_count], &discs[point_count * 2], build);

  // auto discs_unstructured = psl::vector<float>(point_count * sizeof(Disc) / 4);
  // parallel_for(point_count, [&](int i) {
  //   auto disc = discs[i];
  //   discs_unstructured[i * 3 + 0] = disc.p[0];
  //   discs_unstructured[i * 3 + 1] = disc.p[1];
  //   discs_unstructured[i * 3 + 2] = disc.p[2];
  //   discs_unstructured[point_count * 3 + i * 3 + 0] = disc.n[0];
  //   discs_unstructured[point_count * 3 + i * 3 + 1] = disc.n[1];
  //   discs_unstructured[point_count * 3 + i * 3 + 2] = disc.n[2];
  //   discs_unstructured[point_count * 6 + i * 3 + 0] = disc.cd[0];
  //   discs_unstructured[point_count * 6 + i * 3 + 1] = disc.cd[1];
  //   discs_unstructured[point_count * 6 + i * 3 + 2] = disc.cd[2];
  // });
  // static auto ssbo = SSBO(discs_unstructured, 0);
  // program.set_uniform("n_discs", point_count);
  // auto t0 = timer.reset();

  while (!window.should_close()) {
    parallel_for(film.size(), [&](vec2i p) {
      position[p] = {};
      normal[p] = {};
      color[p] = {};
      direct[p] = {};

      auto ray = camera.gen_ray((p + vec2(0.5f)) / film.size());
      auto it = SurfaceInteraction();
      if (accel.intersect(ray, it)) {
        position[p] = it.p;
        normal[p] = it.n;
        auto albedo = it.material().albedo({it, it.n});
        color[p] = albedo;
        if (!accel.hit(spawn_ray1(it.p, it.n, pl - it.p)))
          direct[p] = cl * albedo * absdot(it.n, pl - it.p) / (0.5f + distance_squared(it.p, pl));
      }
    });
    auto t1 = timer.reset();

    debug.set(vec4(0, 0, 0, Infinity));
    auto cursor_p = window.cursor_pos();
    auto pos = position[cursor_p];
    auto nor = normal[cursor_p];
    auto sample = [&](int i) {
      auto Dp = discs[i].p;
      auto Dcd = discs[i].cd;
      auto projected = project(Dp, pos, nor);
      if (!projected) return;
      auto px = int(projected->x * debug.width());
      auto py = int(projected->y * debug.height());

      auto depth = length(Dp - pos);
      if (depth < debug[{px, py}].w) {
        debug[{px, py}] = vec4(Dcd, depth);
      }
    };
    for (int i = point_count; i < point_count * 2; i++) {
      sample(i);
    }

    auto t2 = timer.reset();
    auto debug1 = Texture(debug, 5);
    push_pop(debug1);

    position.upload();
    normal.upload();
    color.upload();
    direct.upload();
    debug.upload();
    debug1.upload();
    auto t3 = timer.reset();

    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    window.update();

    auto t4 = timer.reset();
    static psl::vector<vec4i> buffer;
    buffer.push_back({t1, t2, t3, t4});
    glfwSetWindowTitle(window.ptr(),
                       to_string(mean<vec4i>(trim(reverse_adapter(buffer), 0, 20))).c_str());
  }
}

}  // namespace pine

// pos= camera.position();
//       auto w2n = camera.w2c();
//       auto p = w2n * (Dp - camera.position());
//       if (p.z <= 0) continue;
//       p /= p.z;
//       vec2 p_film = vec2(p) / camera.fov2d() / 2 + vec2(0.5f);
//       auto px = p_film.x * debug.width();
//       auto py = p_film.y * debug.height();
// if (!inside({px, py}, {0, 0}, debug.size)) continue;

// if (window.is_key_pressed(GLFW_KEY_RIGHT))
//   camera.as<ThinLenCamera>().position += camera.c2w().x / 15;
// if (window.is_key_pressed(GLFW_KEY_LEFT))
//   camera.as<ThinLenCamera>().position -= camera.c2w().x / 15;
// if (window.is_key_pressed(GLFW_KEY_UP))
//   camera.as<ThinLenCamera>().position += camera.c2w().z / 15;
// if (window.is_key_pressed(GLFW_KEY_DOWN))
//   camera.as<ThinLenCamera>().position -= camera.c2w().z / 15;
// if (window.is_key_pressed(GLFW_KEY_PAGE_UP))
//   camera.as<ThinLenCamera>().position += camera.c2w().y / 15;
// if (window.is_key_pressed(GLFW_KEY_PAGE_DOWN))
//   camera.as<ThinLenCamera>().position -= camera.c2w().y / 15;
// static auto M = camera.as<ThinLenCamera>().c2w;
// static auto dif_sum = vec2();
// dif_sum += window.cursor_offset();
// camera.as<ThinLenCamera>().c2w = rotate_y(dif_sum.x / 200) * rotate_x(dif_sum.y / 200) * M;