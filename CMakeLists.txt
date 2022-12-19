cmake_minimum_required(VERSION 3.12)

project(pine)

# Pine Library
add_library(pine_lib
src/core/intersection.cpp
src/core/integrator.cpp
src/core/material.cpp
src/core/parallel.cpp
src/core/testing.cpp
src/core/camera.cpp
src/core/shapes.cpp
src/core/string.cpp
src/core/check.cpp
src/core/color.cpp
src/core/image.cpp
src/core/scene.cpp
src/core/math.cpp
src/core/ray.cpp
)
target_include_directories(pine_lib PUBLIC src)
target_compile_features(pine_lib PUBLIC cxx_std_20)
target_compile_options(pine_lib PUBLIC -Wall -Wextra -Wpedantic)

# Pine executable
add_executable(pine
src/cli/pine.cpp
)
target_link_libraries(pine PUBLIC pine_lib)

# Pine test
add_executable(pine_test
src/cli/pine_test.cpp
src/tests/math.cpp
)
target_link_libraries(pine_test PUBLIC pine_lib)
