#include <pine/core/log.h>
#include <pine/core/gui.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <imgui/imgui.h>
#include <imgui/backends/imgui_impl_glfw.h>
#include <imgui/backends/imgui_impl_opengl3.h>

namespace pine {

void launch_gui(psl::string window_name, vec2i window_size, psl::function<void()> draw) {
  glfwSetErrorCallback(
      +[](int, const char* description) { Fatal("[GUI]GLFW Error: ", description); });
  if (!glfwInit())
    Fatal("[GUI]Unable to initialize GLFW");

  auto glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  auto window = glfwCreateWindow(window_size.x, window_size.y, window_name.c_str(), nullptr, nullptr);
  if (window == nullptr)
    Fatal("[GUI]Unable to create window");
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  if (gladLoadGL() == 0)
    Fatal("[GUI]GLAD unable to load OpenGL function address");

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);
  ImGui::GetIO().IniFilename = nullptr;
  ImGui::GetIO().LogFilename = nullptr;

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    draw();

    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  return;
}

}  // namespace pine