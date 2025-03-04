#include <pine/impl/integrator/micro_render.h>
#include <pine/core/parallel.h>
#include <pine/core/profiler.h>
#include <pine/core/opengl.h>
#include <pine/core/scene.h>

namespace pine {

void MicroRenderIntegrator::render(Scene& scene) {
  if (scene.geometries.size() == 0) return;

  Profiler _("[MicroRenderGI]Build");
  auto cdf = Distribution1D();
  for (auto&& geo : scene.geometries) {
    cdf.add_point(geo->area());
  }
  cdf.done();

  auto radius = psl::sqrt(cdf.density_sum() / point_count) / 1.5f;

  for (int i = 0; i < point_count; i++) {
    auto idx = cdf.sample(radical_inverse(0, i)).i;
    auto geo = scene.geometries[idx];
    auto gs = geo->sample({radical_inverse(1, i), radical_inverse(2, i)}, radical_inverse(3, i));
    auto disc = Disc();
    disc[0] = gs.p[0];
    disc[1] = gs.p[1];
    disc[2] = gs.p[2];
    disc[3] = gs.n[0];
    disc[4] = gs.n[1];
    disc[5] = gs.n[2];
    disc[6] = radius;
    auto cd = geo->material->albedo(LeEvalCtx{gs.p, gs.n, gs.uv, gs.n});
    disc[7] = cd[0];
    disc[8] = cd[1];
    disc[9] = cd[2];
    discs.push_back(disc);
  }

  Profiler _a("[MicroRenderGI]Render");
  auto& camera = scene.camera;
  auto& film = scene.camera.film();
  film.set_color(vec4(0, 0, 0, Infinity));

  // auto pixel_SA = 4 * Pi / area(film.size());
  //   auto SA = 2 * Pi * (1 - 1 / psl::sqrt(1 + tan * tan));

  auto window = GLWindow(film.size(), "MicroRenderGI");

  GLuint texture;
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, film.width(), film.height(), 0, GL_RGBA, GL_FLOAT,
               nullptr);
  glBindImageTexture(0, texture, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);

  auto compute_program =
      GLProgram(GLShader(GLShader::Compute, read_string_file("shaders/compute.comp")));
  GLuint ssbo;
  glGenBuffers(1, &ssbo);
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
  glBufferData(GL_SHADER_STORAGE_BUFFER, discs.size() * sizeof(Disc), (float*)discs.data(),
               GL_STATIC_DRAW);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, ssbo);
  compute_program.set_uniform("w2c", camera.w2c());
  compute_program.set_uniform("position", camera.position());
  compute_program.set_uniform("fov2d", camera.fov2d());

  glDispatchCompute(discs.size(), 1, 1);
  glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

  auto program = GLProgram(GLShader(GLShader::Vertex, read_string_file("shaders/basic.vert")),
                           GLShader(GLShader::Fragment, read_string_file("shaders/basic.frag")));
  float vs[]{-1, -1, -1, 1, 1, 1, 1, -1};
  auto vbo = VBO(sizeof(vs), vs);
  auto vao = VAO(vbo);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
  program.use();

  while (!window.should_close()) {
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    window.update();
  }
}

}  // namespace pine