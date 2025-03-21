#include <pine/impl/integrator/path_opengl.h>
#include <pine/core/opengl.h>
#include <pine/core/esl.h>

extern const int sobol_256spp_256d[256 * 256];
extern const int scramblingTile[128 * 128 * 8];
extern const int rankingTile[128 * 128 * 8];

namespace pine {

void OpenGLPathIntegrator::render(Scene& scene) {
  auto film_size = vec2(640, 480);
  auto window = GLWindow(film_size, "Path");
  auto vs = GLShader(GLShader::Vertex, read_string_file("shaders/path.vert"));
  auto fs = GLShader(GLShader::Fragment, load_esl("shaders/path.frag"));
  auto program = GLProgram(vs, fs);

  program.set_uniform("film_size", film_size);
  auto vao = VAO(psl::vector_of<float>(-1, -1, -1, 1, 1, 1, 1, -1), 0, 2);
  auto image = TextureImage<vec4>(film_size, 0);

  auto sobol = SSBO(psl::make_span(sobol_256spp_256d), 10);
  auto sobol_aux0 = SSBO(psl::make_span(scramblingTile), 11);
  auto sobol_aux1 = SSBO(psl::make_span(rankingTile), 12);

  auto cam_position = vec3(5, 1, 0);
  auto cam_c2w = (mat3)look_at(cam_position, {0, 1, 0});
  auto cam_fov = 0.25f;

  struct GPUMaterial {
    vec4 color;
  };
  struct GPUMesh {
    int n_faces;
    int face_index_offset;
    int padding[2];
    GPUMaterial material;
  };
  auto meshes = psl::vector<GPUMesh>();
  auto faces = psl::vector<vec4i>();
  auto vertices = psl::vector<vec4>();
  auto normals = psl::vector<vec4>();
  auto texcoords = psl::vector<vec2>();

  auto face_offset = 0;
  auto n_meshes = 0;
  for (auto&& geo : scene.geometries) {
    if (!geo->shape.is<Mesh>()) continue;
    auto& mesh = geo->shape.as<Mesh>();
    n_meshes++;

    GPUMaterial material_;
    material_.color = (vec4)geo->material->albedo({vec3(0), vec3(0, 0, 1), vec2(0), vec3(0, 0, 1)});
    GPUMesh mesh_;
    mesh_.n_faces = mesh.indices.size();
    mesh_.face_index_offset = faces.size();
    mesh_.material = material_;
    meshes.push_back(mesh_);
    faces.insert_range(faces.end(), psl::transform(mesh.indices, [&](vec3i face) {
                         return vec4i(face + vec3i(face_offset));
                       }));
    vertices.insert_range(vertices.end(),
                          psl::transform(mesh.vertices, psl::converter<vec3, vec4>));
    normals.insert_range(normals.end(), psl::transform(mesh.normals, psl::converter<vec3, vec4>));
    texcoords.insert_range(texcoords.end(), mesh.texcoords);
    face_offset += mesh.vertices.size();
  }

  program.set_uniform("n_meshes", n_meshes);
  auto meshes_g = SSBO(meshes, 0);
  auto faces_g = SSBO(faces, 1);
  auto vertices_g = SSBO(vertices, 2);
  auto normals_g = SSBO(normals, 3);
  auto texcoords_g = SSBO(texcoords, 4);

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
    if (window.is_key_pressed(GLFW_KEY_PAGE_UP)) cam_position += cam_c2w.y * speed / 2, alpha = 0;
    if (window.is_key_pressed(GLFW_KEY_PAGE_DOWN)) cam_position -= cam_c2w.y * speed / 2, alpha = 0;
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

    glDrawArrays(GL_TRIANGLE_FAN, 0, 6);

    static bool key_k_active = true;
    if (window.is_key_pressed(GLFW_KEY_K)) {
      if (key_k_active) {
        key_k_active = false;
        LOG("Saving screenshot");
        auto pixels = Array2d4f(film_size);
        glReadPixels(0, 0, film_size.x, film_size.y, GL_RGBA, GL_FLOAT, pixels.data());
        save_image("images/screenshot.png", pixels, true);
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