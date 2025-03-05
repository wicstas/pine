#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>

#include <psl/array.h>

#include <pine/core/vecmath.h>
#include <pine/core/fileio.h>
#include <pine/core/log.h>

namespace pine {

struct GLWindow {
  GLWindow(vec2i size, psl::string title) {
    glfwSetErrorCallback(
        +[](int, const char* description) { SEVERE("GLFW Error: ", description); });
    if (!glfwInit()) SEVERE("Unable to initialize GLFW");

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_SAMPLES, 4);

    window = glfwCreateWindow(size.x, size.y, title.c_str(), nullptr, nullptr);
    if (window == nullptr) SEVERE("GLFW: unable to create window");
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
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

  void update() const {
    glfwPollEvents();

    glfwSwapBuffers(window);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }

  bool should_close() const { return glfwWindowShouldClose(window); }

  bool is_key_pressed(int key) const { return glfwGetKey(window, key) == GLFW_PRESS; }
  vec2 cursor_pos() const {
    double x, y;
    glfwGetCursorPos(window, &x, &y);
    return vec2(x, y);
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
  ~GLProgram() { glDeleteProgram(program); }
  GLProgram(GLProgram&&) = delete;

  void use() const { glUseProgram(program); }

  void set_uniform(const char* name, int value) const {
    auto loc = glGetUniformLocation(program, name);
    if (loc == -1) SEVERE("GLProgram: uniform `", name, "` not found");
    glUniform1i(loc, value);
  }
  void set_uniform(const char* name, float value) const {
    auto loc = glGetUniformLocation(program, name);
    if (loc == -1) SEVERE("GLProgram: uniform `", name, "` not found");
    glUniform1f(loc, value);
  }
  void set_uniform(const char* name, vec2 value) const {
    auto loc = glGetUniformLocation(program, name);
    if (loc == -1) SEVERE("GLProgram: uniform `", name, "` not found");
    glUniform2f(loc, value[0], value[1]);
  }
  void set_uniform(const char* name, vec2i value) const {
    auto loc = glGetUniformLocation(program, name);
    if (loc == -1) SEVERE("GLProgram: uniform `", name, "` not found");
    glUniform2i(loc, value[0], value[1]);
  }
  void set_uniform(const char* name, vec3 value) const {
    auto loc = glGetUniformLocation(program, name);
    if (loc == -1) SEVERE("GLProgram: uniform `", name, "` not found");
    glUniform3f(loc, value[0], value[1], value[2]);
  }
  void set_uniform(const char* name, mat4 value) const {
    auto loc = glGetUniformLocation(program, name);
    if (loc == -1) SEVERE("GLProgram: uniform `", name, "` not found");
    glUniformMatrix4fv(loc, 1, false, &value[0][0]);
  }
  void set_uniform(const char* name, mat3 value) const {
    auto loc = glGetUniformLocation(program, name);
    if (loc == -1) SEVERE("GLProgram: uniform `", name, "` not found");
    glUniformMatrix3fv(loc, 1, false, &value[0][0]);
  }
  GLuint program;
};

struct VBO {
  VBO(size_t size, void* data) {
    glCreateBuffers(1, &vbo);
    bind();
    glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);
  }
  ~VBO() { glDeleteBuffers(1, &vbo); }
  VBO(VBO&& rhs) : vbo(psl::exchange(rhs.vbo, 0)) {}

  void bind() const { glBindBuffer(GL_ARRAY_BUFFER, vbo); }

  GLuint vbo = 0;
};

struct VAO {
  VAO(const VBO& vbo, int index, int size) {
    vbo.bind();
    glCreateVertexArrays(1, &vao);
    bind();
    glEnableVertexAttribArray(index);
    glVertexAttribPointer(index, size, GL_FLOAT, GL_FALSE, 0, nullptr);
  }
  ~VAO() { glDeleteVertexArrays(1, &vao); }
  VAO(VAO&& rhs) : vao(psl::exchange(rhs.vao, 0)) {}

  void bind() const { glBindVertexArray(vao); }

  GLuint vao = 0;
};

struct SSBO {
  SSBO(size_t size, void* data, int index) {
    glGenBuffers(1, &ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, size, data, GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, index, ssbo);
  }

  GLuint ssbo;
};

}  // namespace pine