#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>

#include <psl/array.h>

#include <pine/core/vecmath.h>
#include <pine/core/fileio.h>
#include <pine/core/log.h>

namespace pine {

struct GLWindow {
  GLWindow(vec2i size, psl::string title) : size(size) {
    glfwSetErrorCallback(
        +[](int, const char* description) { SEVERE("GLFW Error: ", description); });
    if (!glfwInit()) SEVERE("Unable to initialize GLFW");

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_SAMPLES, 4);

    window = glfwCreateWindow(size.x, size.y, title.c_str(), nullptr, nullptr);
    if (window == nullptr) SEVERE("GLFW: unable to create window");
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);
    // glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    if (gladLoadGL() == 0) SEVERE("GLAD: unable to load GL function address");

    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(
        [](GLenum source, GLenum, GLuint, GLenum severity, GLsizei, const GLchar* message,
           const void*) {
          if (source != GL_DEBUG_SOURCE_SHADER_COMPILER)
            if (severity == GL_DEBUG_SEVERITY_MEDIUM || severity == GL_DEBUG_SEVERITY_HIGH)
              SEVERE("GL: ", message);
        },
        0);

    glEnable(GL_MULTISAMPLE);
    glClearColor(0, 0, 0, 0);
  }
  ~GLWindow() { glfwDestroyWindow(window); }
  GLWindow(GLWindow&&) = delete;

  void update(bool clear_framebuffer = true) const {
    glfwPollEvents();

    glfwSwapBuffers(window);
    if (clear_framebuffer) glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }

  bool should_close() const { return glfwWindowShouldClose(window); }

  bool is_key_pressed(int key) const { return glfwGetKey(window, key) == GLFW_PRESS; }
  vec2 cursor_pos() const {
    double x, y;
    glfwGetCursorPos(window, &x, &y);
    y = size.y - 1 - y;
    return {x, y};
  }
  vec2 cursor_offset() const {
    auto p = cursor_pos();
    auto offset = last_cursor_pos ? p - *last_cursor_pos : vec2();
    last_cursor_pos = p;
    return offset;
  }

  GLFWwindow* ptr() { return window; }

 private:
  GLFWwindow* window;
  mutable psl::optional<vec2> last_cursor_pos;
  vec2i size;
};

struct GLShader {
  enum Type { Vertex, Fragment, Compute };
  GLShader(Type type, psl::string content) {
    switch (type) {
      case Vertex:
        shader = glCreateShader(GL_VERTEX_SHADER);
        break;
      case Fragment:
        shader = glCreateShader(GL_FRAGMENT_SHADER);
        break;
      case Compute:
        shader = glCreateShader(GL_COMPUTE_SHADER);
        break;
      default:
        PINE_UNREACHABLE;
    }
    auto length = int(content.size());
    const char* content_array[]{content.c_str()};
    glShaderSource(shader, 1, content_array, &length);
    glCompileShader(shader);

    char info[1024];
    GLsizei info_length;
    glGetShaderInfoLog(shader, 1024, &info_length, info);
    if (info_length) SEVERE("GL shader:\n", info);
  }
  ~GLShader() { glDeleteShader(shader); }
  GLShader(GLShader&&) = delete;

  GLuint shader;
};

struct GLProgram {
  GLProgram(const GLShader& compute_shader) {
    program = glCreateProgram();
    glAttachShader(program, compute_shader.shader);
    glLinkProgram(program);

    char info[1024];
    GLsizei info_length;
    glGetProgramInfoLog(program, 1024, &info_length, info);
    if (info_length) SEVERE("GL program: ", info);

    glDetachShader(program, compute_shader.shader);

    glUseProgram(program);
  }
  GLProgram(const GLShader& vshader, const GLShader& fshader) {
    program = glCreateProgram();
    glAttachShader(program, vshader.shader);
    glAttachShader(program, fshader.shader);
    glLinkProgram(program);

    char info[1024];
    GLsizei info_length;
    glGetProgramInfoLog(program, 1024, &info_length, info);
    if (info_length) SEVERE("GL program: ", info);

    glDetachShader(program, vshader.shader);
    glDetachShader(program, fshader.shader);

    glUseProgram(program);
  }
  GLProgram(psl::string_view vs_path, psl::string_view fs_path)
      : GLProgram(GLShader(GLShader::Vertex, read_string_file(vs_path)),
                  GLShader(GLShader::Fragment, read_string_file(fs_path))) {}
  ~GLProgram() { glDeleteProgram(program); }
  GLProgram(GLProgram&&) = delete;

  void use() const { glUseProgram(program); }

  void set_uniform(const char* name, int value) const {
    auto loc = glGetUniformLocation(program, name);
    if (loc == -1) WARNING("GLProgram: uniform `", name, "` not found");
    glUniform1i(loc, value);
  }
  void set_uniform(const char* name, float value) const {
    auto loc = glGetUniformLocation(program, name);
    if (loc == -1) WARNING("GLProgram: uniform `", name, "` not found");
    glUniform1f(loc, value);
  }
  void set_uniform(const char* name, vec2 value) const {
    auto loc = glGetUniformLocation(program, name);
    if (loc == -1) WARNING("GLProgram: uniform `", name, "` not found");
    glUniform2f(loc, value[0], value[1]);
  }
  void set_uniform(const char* name, vec2i value) const {
    auto loc = glGetUniformLocation(program, name);
    if (loc == -1) WARNING("GLProgram: uniform `", name, "` not found");
    glUniform2i(loc, value[0], value[1]);
  }
  void set_uniform(const char* name, vec3 value) const {
    auto loc = glGetUniformLocation(program, name);
    if (loc == -1) WARNING("GLProgram: uniform `", name, "` not found");
    glUniform3f(loc, value[0], value[1], value[2]);
  }
  void set_uniform(const char* name, mat4 value) const {
    auto loc = glGetUniformLocation(program, name);
    if (loc == -1) WARNING("GLProgram: uniform `", name, "` not found");
    glUniformMatrix4fv(loc, 1, false, &value[0][0]);
  }
  void set_uniform(const char* name, mat3 value) const {
    auto loc = glGetUniformLocation(program, name);
    if (loc == -1) WARNING("GLProgram: uniform `", name, "` not found");
    glUniformMatrix3fv(loc, 1, false, &value[0][0]);
  }
  GLuint program;
};

struct VAO {
  VAO(psl::span<const float> data, int index, int attrib_size) {
    glCreateBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    auto size = sizeof(float) * data.size();
    glBufferData(GL_ARRAY_BUFFER, size, &data[0], GL_STATIC_DRAW);

    glCreateVertexArrays(1, &vao);
    bind();
    glEnableVertexAttribArray(index);
    glVertexAttribPointer(index, attrib_size, GL_FLOAT, GL_FALSE, 0, nullptr);
  }
  ~VAO() {
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
  }
  VAO(VAO&& rhs) : vbo(psl::exchange(rhs.vbo, 0)), vao(psl::exchange(rhs.vao, 0)) {}

  void bind() const { glBindVertexArray(vao); }

  GLuint vbo = 0;
  GLuint vao = 0;
};

struct SSBO {
  SSBO(size_t size, const void* data, int index) : size(size) {
    glGenBuffers(1, &ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, size, data, GL_DYNAMIC_READ);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, index, ssbo);
  }
  template <typename T>
  SSBO(psl::span<T> data, int index) : SSBO(data.size() * sizeof(T), &data[0], index) {}
  template <typename T>
  SSBO(const psl::vector<T>& data, int index) : SSBO(psl::span<const T>(data), index) {}

  void update(void* data) {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, size, data);
  }

  size_t size;
  GLuint ssbo;
};

template <typename T>
struct Texture {
  Texture(vec2i size, int index) : texels(size), size(size), index(index) {
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, internal_type(), size.x, size.y, 0, format(), base_type(),
                 nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glActiveTexture(GL_TEXTURE0 + index);
    glBindTexture(GL_TEXTURE_2D, texture);
  };
  Texture(const Texture& rhs, int index) : Texture(rhs.size, index) { texels = rhs.texels; }
  void upload() {
    glActiveTexture(GL_TEXTURE0 + index);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, size.x, size.y, format(), base_type(), texels.data());
  }
  auto& operator[](vec2i p) { return texels[p]; }
  void set(T value) { texels.set(value); }

  void bind() { glBindTexture(GL_TEXTURE_2D, texture); }
  static int internal_type() {
    if (psl::same_as<T, vec3>)
      return GL_RGB32F;
    else if (psl::same_as<T, vec4>)
      return GL_RGBA32F;
    else
      PINE_UNREACHABLE;
  }
  static int format() {
    if (psl::same_as<T, vec3>)
      return GL_RGB;
    else if (psl::same_as<T, vec4>)
      return GL_RGBA;
    else
      PINE_UNREACHABLE;
  }
  static int base_type() { return GL_FLOAT; }

  int width() const { return size.x; }
  int height() const { return size.y; }

  GLuint texture;
  Array2d<T> texels;
  vec2i size;
  int index;
};

template <typename T>
struct Texture2D {
  Texture2D(vec2i size, int index) : size(size), index(index) {
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, internal_type(), size.x, size.y, 0, format(), base_type(),
                 nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, texture);
    glBindImageTexture(index, texture, 0, false, 0, GL_READ_ONLY, internal_type());
  };
  static int internal_type() {
    if (psl::same_as<T, vec3>)
      return GL_RGB32F;
    else if (psl::same_as<T, vec4>)
      return GL_RGBA32F;
    else
      PINE_UNREACHABLE;
  }
  static int format() {
    if (psl::same_as<T, vec3>)
      return GL_RGB;
    else if (psl::same_as<T, vec4>)
      return GL_RGBA;
    else
      PINE_UNREACHABLE;
  }
  static int base_type() { return GL_FLOAT; }

  int width() const { return size.x; }
  int height() const { return size.y; }

  GLuint texture;
  vec2i size;
  int index;
};

template <typename T>
struct FBO {
  FBO(vec2i size, int index) : size(size), index(index) {
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, internal_type(), size.x, size.y, 0, format(), base_type(),
                 nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glActiveTexture(GL_TEXTURE0 + index);
    glBindTexture(GL_TEXTURE_2D, texture);
    glBindImageTexture(index, texture, 0, false, 0, GL_READ_ONLY, internal_type());

    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);
    CHECK(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
  }
  void bind() {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  }
  static void bind_main() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }

  static int internal_type() {
    if (psl::same_as<T, vec3>)
      return GL_RGB32F;
    else if (psl::same_as<T, vec4>)
      return GL_RGBA32F;
    else
      PINE_UNREACHABLE;
  }
  static int format() {
    if (psl::same_as<T, vec3>)
      return GL_RGB;
    else if (psl::same_as<T, vec4>)
      return GL_RGBA;
    else
      PINE_UNREACHABLE;
  }
  static int base_type() { return GL_FLOAT; }

  int width() const { return size.x; }
  int height() const { return size.y; }

  GLuint texture;
  GLuint fbo;
  vec2i size;
  int index;
};

template <typename T>
struct TextureImage {
  TextureImage(vec2i size, int index) : size(size), index(index) {
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, internal_type(), size.x, size.y, 0, format(), base_type(),
                 nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glActiveTexture(GL_TEXTURE0 + index);
    glBindTexture(GL_TEXTURE_2D, texture);
    glBindImageTexture(index, texture, 0, false, 0, GL_READ_WRITE, internal_type());
  }

  static int internal_type() {
    if (psl::same_as<T, vec3>)
      return GL_RGB32F;
    else if (psl::same_as<T, vec4>)
      return GL_RGBA32F;
    else
      PINE_UNREACHABLE;
  }
  static int format() {
    if (psl::same_as<T, vec3>)
      return GL_RGB;
    else if (psl::same_as<T, vec4>)
      return GL_RGBA;
    else
      PINE_UNREACHABLE;
  }
  static int base_type() { return GL_FLOAT; }

  int width() const { return size.x; }
  int height() const { return size.y; }

  GLuint texture;
  GLuint fbo;
  vec2i size;
  int index;
};

template <typename T>
struct Texture1D {
  Texture1D(const psl::vector<T>& data, int index) : index(index) {
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_1D, texture);
    glTexImage1D(GL_TEXTURE_1D, 0, internal_type(), data.size(), 0, format(), base_type(),
                 data.data());
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glActiveTexture(GL_TEXTURE0 + index);
    glBindTexture(GL_TEXTURE_1D, texture);
    glBindImageTexture(index, texture, 0, false, 0, GL_READ_ONLY, internal_type());
  };

  static int internal_type() {
    if (psl::same_as<T, vec3>)
      return GL_RGB32F;
    else if (psl::same_as<T, vec4>)
      return GL_RGBA32F;
    else if (psl::same_as<T, vec3i>)
      return GL_RGB32I;
    else if (psl::same_as<T, vec4i>)
      return GL_RGBA32I;
    else
      PINE_UNREACHABLE;
  }
  static int format() {
    if (psl::same_as<T, vec3>)
      return GL_RGB;
    else if (psl::same_as<T, vec4>)
      return GL_RGBA;
    else if (psl::same_as<T, vec3i>)
      return GL_RGB_INTEGER;
    else if (psl::same_as<T, vec4i>)
      return GL_RGBA_INTEGER;
    else
      PINE_UNREACHABLE;
  }
  static int base_type() {
    if (psl::same_as<T, vec3>)
      return GL_FLOAT;
    else if (psl::same_as<T, vec4>)
      return GL_FLOAT;
    else if (psl::same_as<T, vec3i>)
      return GL_INT;
    else if (psl::same_as<T, vec4i>)
      return GL_INT;
    else
      PINE_UNREACHABLE;
  }

  GLuint texture;
  int index;
};

}  // namespace pine