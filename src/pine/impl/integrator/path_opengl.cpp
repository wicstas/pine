#include <pine/impl/integrator/path_opengl.h>
#include <pine/core/opengl.h>
#include <pine/core/esl.h>

extern const int sobol_256spp_256d[256 * 256];
extern const int scramblingTile[128 * 128 * 8];
extern const int rankingTile[128 * 128 * 8];

namespace pine {

void OpenGLPathIntegrator::render() {
  auto film_size = vec2(640, 480);
  auto window = GLWindow(film_size, "Path");
  auto vs = GLShader(GLShader::Vertex, read_string_file("shaders/path.vert"));
  auto fs = GLShader(GLShader::Fragment, load_esl("shaders/path.frag"));
  auto program = GLProgram(vs, fs);

  program.set_uniform("film_size", film_size);
  auto vao = VAO(psl::vector_of<float>(-1, -1, -1, 1, 1, 1, 1, -1), 0, 2);
  auto fbo = FBO<vec4>(film_size, 0);
  auto image = Array2d4f(film_size);

  auto sobol = SSBO(psl::make_span(sobol_256spp_256d), 10);
  auto sobol_aux0 = SSBO(psl::make_span(scramblingTile), 11);
  auto sobol_aux1 = SSBO(psl::make_span(rankingTile), 12);

  auto cam_position = vec3(5, 1, 0);
  auto cam_c2w = (mat3)look_at(cam_position, {0, 1, 0});
  auto cam_fov = 0.25f;

  auto& mesh = meshes[0];
  program.set_uniform("mesh.n_faces", (int)mesh.indices.size());
  program.set_uniform("mesh.offset_face", 0);
  auto faces = SSBO(psl::transform_vector(mesh.indices, [](vec3i v) { return vec4i(v); }), 0);
  auto vertices = SSBO(psl::transform_vector(mesh.vertices, [](vec3 v) { return vec4(v); }), 1);
  auto normals = SSBO(psl::transform_vector(mesh.normals, [](vec3 v) { return vec4(v); }), 2);
  auto texcoords = SSBO(mesh.texcoords, 3);

  auto timer = Timer();
  auto speed = 0.07f;
  auto sensitivity = 0.005f;
  auto alpha = 0;
  while (!window.should_close()) {
    if (window.is_key_pressed(GLFW_KEY_LEFT_SHIFT))
      speed = 0.2f;
    else
      speed = 0.07f;

    if (window.is_key_pressed(GLFW_KEY_RIGHT)) cam_position += cam_c2w.x * speed, alpha = 0;
    if (window.is_key_pressed(GLFW_KEY_LEFT)) cam_position -= cam_c2w.x * speed, alpha = 0;
    if (window.is_key_pressed(GLFW_KEY_UP)) cam_position += cam_c2w.z * speed, alpha = 0;
    if (window.is_key_pressed(GLFW_KEY_DOWN)) cam_position -= cam_c2w.z * speed, alpha = 0;
    if (window.is_key_pressed(GLFW_KEY_PAGE_UP)) cam_position += cam_c2w.y * speed, alpha = 0;
    if (window.is_key_pressed(GLFW_KEY_PAGE_DOWN)) cam_position -= cam_c2w.y * speed, alpha = 0;
    if (window.is_key_pressed(GLFW_KEY_E)) cam_fov -= 0.01f, alpha = 0;
    if (window.is_key_pressed(GLFW_KEY_Q)) cam_fov += 0.01f, alpha = 0;
    static auto dif_sum = vec2();
    if (auto offset = window.cursor_offset(); offset != vec2()) {
      dif_sum += offset;
      alpha = 0;
    }
    static auto M = cam_c2w;
    cam_c2w = M * rotate_y(dif_sum.x * sensitivity) * rotate_x(-dif_sum.y * sensitivity);
    program.set_uniform("cam.position", cam_position);
    program.set_uniform("cam.c2w", cam_c2w);
    program.set_uniform("cam.fov2d", vec2(film_size.x / film_size.y * cam_fov, cam_fov));
    program.set_uniform("alpha", alpha);

    fbo.bind();
    glDrawArrays(GL_TRIANGLE_FAN, 0, 6);

    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo.fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, film_size.x, film_size.y, 0, 0, film_size.x, film_size.y,
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);

    static bool key_k_active = true;
    if (window.is_key_pressed(GLFW_KEY_K)) {
      if (key_k_active) {
        key_k_active = false;
        LOG("Saving screenshot");
        glReadPixels(0, 0, film_size.x, film_size.y, GL_RGBA, GL_FLOAT, image.data());
        save_image("images/screenshot.png", image, true);
      }
    } else {
      key_k_active = true;
    }

    window.update();
    alpha++;
    glfwSetWindowTitle(
        window.ptr(), psl::to_string(alpha, " / ", int(1000.0f / timer.reset()), " spp/s").c_str());
  }
}

}  // namespace pine