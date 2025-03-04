cmake_minimum_required(VERSION 3.16)

project(pine LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(BUILD_SHARED_LIBS OFF)

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
target_compile_options(psl PRIVATE -Wall -Wextra -pedantic)
set_target_properties(psl PROPERTIES POSITION_INDEPENDENT_CODE ON)
#target_compile_options(psl PUBLIC $<$<CONFIG:Debug>: -g -fsanitize=address -fno-omit-frame-pointer>)
#target_link_options(psl PUBLIC $<$<CONFIG:Debug>:-g -fsanitize=address>)

#psl Test
# add_executable(psl_test test/psl_test.cpp)
# target_link_libraries(psl_test psl)

#Pine Core Library
add_library(pinelib
src/pine/core/program_context.cpp
src/pine/core/phase_function.cpp
src/pine/core/lowdiscrepancy.cpp
src/pine/core/sobolmatrices.cpp
src/pine/core/jit_compiler.cpp
src/pine/core/lightsampler.cpp
src/pine/core/distribution.cpp
src/pine/core/video_writer.cpp
src/pine/core/interaction.cpp
src/pine/core/integrator.cpp
src/pine/core/blackbody.cpp
src/pine/core/geometry.cpp
src/pine/core/material.cpp
src/pine/core/profiler.cpp
src/pine/core/parallel.cpp
#src/pine/core/compiler.cpp
src/pine/core/context.cpp
src/pine/core/vecmath.cpp
src/pine/core/sampler.cpp
src/pine/core/denoise.cpp
src/pine/core/primes.cpp
src/pine/core/fileio.cpp
src/pine/core/camera.cpp
src/pine/core/medium.cpp
src/pine/core/noise.cpp
src/pine/core/scene.cpp
src/pine/core/light.cpp
src/pine/core/accel.cpp
src/pine/core/array.cpp
src/pine/core/color.cpp
src/pine/core/image.cpp
src/pine/core/bbox.cpp
src/pine/core/node.cpp
src/pine/core/film.cpp
src/pine/core/bxdf.cpp
src/pine/core/math.cpp
src/pine/core/rng.cpp
src/pine/core/log.cpp
src/pine/impl/integrator/micro_render.cpp
src/pine/impl/integrator/randomwalk.cpp
src/pine/impl/integrator/guidedpath.cpp
src/pine/impl/integrator/cachedpath.cpp
src/pine/impl/integrator/denoiser.cpp
src/pine/impl/integrator/restir.cpp
src/pine/impl/integrator/mlt.cpp
src/pine/impl/integrator/path.cpp
src/pine/impl/integrator/ao.cpp
src/pine/impl/accel/embree.cpp
src/pine/impl/accel/bvh.cpp


src/contrib/libgwavi/avi-utils.cpp
src/contrib/libgwavi/fileio.cpp
src/contrib/libgwavi/gwavi.cpp
src/contrib/stb_image_write.cpp
src/contrib/stb_image.cpp
src/contrib/tiny_gltf.cpp
src/contrib/glad/glad.c

src/contrib/bluesobol/bluenoise_1spp.cpp
src/contrib/bluesobol/bluenoise_2spp.cpp
src/contrib/bluesobol/bluenoise_4spp.cpp
src/contrib/bluesobol/bluenoise_8spp.cpp
src/contrib/bluesobol/bluenoise_16spp.cpp
src/contrib/bluesobol/bluenoise_32spp.cpp
src/contrib/bluesobol/bluenoise_64spp.cpp
src/contrib/bluesobol/bluenoise_128spp.cpp
src/contrib/bluesobol/bluenoise_256spp.cpp
)

target_compile_options(pinelib PRIVATE -Wall -Wextra -pedantic)
target_include_directories(pinelib PUBLIC src/)
target_include_directories(pinelib PRIVATE src/contrib)
set_target_properties(pinelib PROPERTIES POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)
find_package(Embree REQUIRED)
find_package(LLVM 18.1.8 REQUIRED CONFIG)
target_include_directories(pinelib PRIVATE ${LLVM_INCLUDE_DIRS})
find_package(glfw3 REQUIRED)
target_link_libraries(pinelib psl Threads::Threads embree LLVM-18 glfw)

#Pine executable
add_executable(pine src/cli/pine.cpp)
target_link_libraries(pine pinelib)

#Gltf tool
# add_executable(gltf-tool src/cli/gltf-tool.cpp)
# target_link_libraries(gltf-tool pinelib)

#Image tool
#add_executable(image-tool src/cli/image-tool.cpp)
#target_link_libraries(image-tool pinelib)

#Pine executable
#add_executable(pine-repl src/cli/pine-repl.cpp)
#target_link_libraries(pine-repl pinelib ncurses)