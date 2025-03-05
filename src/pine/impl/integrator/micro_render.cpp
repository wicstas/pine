#include <pine/impl/integrator/micro_render.h>
#include <pine/core/parallel.h>
#include <pine/core/profiler.h>
#include <pine/core/opengl.h>
#include <pine/core/scene.h>

namespace pine {

auto pl = vec3(0, 1.9, 1);
auto cl = vec3(1, 1, 1) / 2;

void MicroRenderIntegrator::render(Scene& scene) {
  if (scene.geometries.size() == 0) return;
  auto& camera = scene.camera;
  auto& film = scene.camera.film();
  auto accel = EmbreeAccel(&scene);

  auto cdf = Distribution1D(transform_vector(scene.geometries, [](auto& x) { return x->area(); }));
  auto radius = psl::sqrt(cdf.density_sum() / point_count);

  auto window = GLWindow(film.size(), "MicroRenderGI");
  auto program = GLProgram(GLShader(GLShader::Vertex, read_string_file("shaders/basic.vert")),
                           GLShader(GLShader::Fragment, read_string_file("shaders/mrgi.frag")));
  float vs[]{-1, -1, -1, 1, 1, 1, 1, -1};
  auto vbo = VBO(sizeof(vs), vs);
  auto vao = VAO(vbo, 0, 2);

  program.set_uniform("film_size", film.size());
  auto position = Array2d3f(film.size());
  auto normal = Array2d3f(film.size());
  auto color = Array2d3f(film.size());
  auto direct = Array2d3f(film.size());
  auto setup_texture = [](void* data, int width, int height, int index) {
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width, height, 0, GL_RGB, GL_FLOAT, data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glActiveTexture(GL_TEXTURE0 + index);
    glBindTexture(GL_TEXTURE_2D, texture);
    return texture;
  };
  GLuint Tp = setup_texture(position.data(), film.width(), film.height(), 0);
  GLuint Tn = setup_texture(normal.data(), film.width(), film.height(), 1);
  GLuint Tcd = setup_texture(color.data(), film.width(), film.height(), 2);
  GLuint Tdirect = setup_texture(direct.data(), film.width(), film.height(), 3);

  for (int i = 0; i < point_count; i++) {
    auto idx = cdf.sample(radical_inverse(0, i)).i;
    auto geo = scene.geometries[idx];
    auto gs = geo->sample({radical_inverse(1, i), radical_inverse(2, i)}, radical_inverse(3, i));
    auto cd = vec3(0);
    if (!accel.hit(spawn_ray1(gs.p, gs.n, pl - gs.p)))
      cd = cl * geo->material->albedo({gs.p, gs.n, gs.uv, gs.n}) /
           (distance_squared(gs.p, pl) + .5f);
    discs.emplace_back(gs.p, gs.n, radius, cd);
  }
  auto discs_ssbo = SSBO(discs.size() * sizeof(Disc), discs.data(), 0);
  program.set_uniform("n_discs", int(discs.size()));

  auto timer = Timer();
  psl::vector<float> buffer;

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
          direct[p] = cl * albedo / (distance_squared(it.p, pl) + .5f);
      }
    });

    glBindTexture(GL_TEXTURE_2D, Tp);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, film.width(), film.height(), GL_RGB, GL_FLOAT,
                    position.data());
    glBindTexture(GL_TEXTURE_2D, Tn);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, film.width(), film.height(), GL_RGB, GL_FLOAT,
                    normal.data());
    glBindTexture(GL_TEXTURE_2D, Tcd);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, film.width(), film.height(), GL_RGB, GL_FLOAT,
                    color.data());
    glBindTexture(GL_TEXTURE_2D, Tdirect);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, film.width(), film.height(), GL_RGB, GL_FLOAT,
                    direct.data());

    timer.reset();
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    window.update();
    buffer.push_back(timer.elapsed_ms());
    glfwSetWindowTitle(window.ptr(),
                       psl::to_string(mean<float>(trim(reverse_adapter(buffer), 0, 20))).c_str());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }
}

}  // namespace pine