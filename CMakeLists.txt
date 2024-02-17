cmake_minimum_required(VERSION 3.12)

project(pine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

add_compile_options(-Wall -Wextra -pedantic -ffast-math)

#psl Library
add_library(psl
src/psl/iostream.cpp
src/psl/fstream.cpp
src/psl/string.cpp
src/psl/chrono.cpp
src/psl/memory.cpp
src/psl/system.cpp
src/psl/check.cpp
)
target_include_directories(psl PUBLIC src/)
#target_compile_options(psl PUBLIC $<$<CONFIG:Debug>:-g -fsanitize=address -fno-omit-frame-pointer>)
#target_link_options(psl PUBLIC $<$<CONFIG:Debug>:-g -fsanitize=address>)

#Pine Core Library
add_library(pinelib
src/pine/core/lowdiscrepancy.cpp
src/pine/core/lightsampler.cpp
src/pine/core/distribution.cpp
src/pine/core/interaction.cpp
src/pine/core/integrator.cpp
src/pine/core/voxelizer.cpp
src/pine/core/geometry.cpp
src/pine/core/material.cpp
src/pine/core/profiler.cpp
src/pine/core/parallel.cpp
src/pine/core/compiler.cpp
src/pine/core/context.cpp
src/pine/core/vecmath.cpp
src/pine/core/sampler.cpp
src/pine/core/primes.cpp
src/pine/core/fileio.cpp
src/pine/core/parser.cpp
src/pine/core/camera.cpp
src/pine/core/noise.cpp
src/pine/core/scene.cpp
src/pine/core/light.cpp
src/pine/core/accel.cpp
src/pine/core/array.cpp
src/pine/core/color.cpp
src/pine/core/image.cpp
src/pine/core/aabb.cpp
src/pine/core/node.cpp
src/pine/core/film.cpp
src/pine/core/bxdf.cpp
src/pine/core/math.cpp
src/pine/core/rng.cpp
src/pine/core/log.cpp
#src/pine/impl/integrator/spatial_tree.cpp
src/pine/impl/integrator/randomwalk.cpp
src/pine/impl/integrator/visualizer.cpp
src/pine/impl/integrator/guidedpath.cpp
src/pine/impl/integrator/cachedpath.cpp
src/pine/impl/integrator/voxelcone.cpp
src/pine/impl/integrator/path.cpp
src/pine/impl/integrator/ao.cpp
src/pine/impl/accel/embree.cpp
src/pine/impl/accel/bvh.cpp
)

target_include_directories(pinelib PUBLIC src/)
set_target_properties(pinelib PROPERTIES POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)

set(BUILD_SHARED_LIBS OFF)
set(ASSIMP_NO_EXPORT ON)
set(ASSIMP_BUILD_TESTS OFF)  
set(ASSIMP_INSTALL OFF)  
set(ASSIMP_INSTALL_PDB OFF)
add_subdirectory(src/contrib/assimp)

FIND_PACKAGE(embree 4 REQUIRED)

target_link_libraries(pinelib psl Threads::Threads assimp embree)

#Pine CommandLine
add_executable(pine src/cli/main.cpp)
target_link_libraries(pine pinelib)

#psl Test
add_executable(psl_test test/psl_test.cpp)
target_link_libraries(psl_test psl)