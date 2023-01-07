cmake_minimum_required(VERSION 3.18)

project(pine)

add_subdirectory(src/ext/pybind11)

# Pine Library
add_library(pine_lib
src/core/intersection.cpp
src/core/renderer.cpp
src/core/material.cpp
src/core/parallel.cpp
src/core/logging.cpp
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
target_compile_options(pine_lib PUBLIC -fPIC -Wall -Wextra -Wpedantic)
target_compile_definitions(pine_lib PUBLIC $<$<NOT:$<CONFIG:Release>>: PINE_DEBUG_BUILD>)

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

pybind11_add_module(pinepy src/core/python.cpp)
target_link_libraries(pinepy PUBLIC pine_lib)