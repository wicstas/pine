#pragma once
#include <pine/core/vecmath.h>

#include <psl/function.h>

struct GLFWwindow;

namespace pine {

void launch_gui(psl::string window_name, vec2i window_size, psl::function<void()> draw);

}  // namespace pine
