#pragma once

#include <pine/core/geometry.h>
#include <pine/core/image.h>

#include <psl/fstream.h>
#include <psl/string.h>
#include <psl/vector.h>
#include <psl/memory.h>

namespace pine {

psl::string read_string_file(psl::string_view filename);

void write_binary_file(psl::string_view filename, const void* ptr, size_t size);
psl::vector<char> read_binary_file(psl::string_view filename);

psl::vector<uint8_t> to_uint8_array(vec2i size, int nchannel, const float* data);

void save_image(psl::string filename, vec2i size, int nchannel, const float* data);
void save_image(psl::string filename, vec2i size, int nchannel, const uint8_t* data);
inline void save_image(psl::string filename, const Array2D<vec2>& pixels) {
  save_image(filename, pixels.size(), 2, &pixels.data()[0][0]);
}
inline void save_image(psl::string filename, const Array2D<vec3>& pixels) {
  save_image(filename, pixels.size(), 3, &pixels.data()[0][0]);
}
inline void save_image(psl::string filename, const Array2D<vec4>& pixels) {
  save_image(filename, pixels.size(), 4, &pixels.data()[0][0]);
}

Image read_image(psl::string_view filename);
Image read_image(void* data, size_t size);

TriangleMesh load_mesh(void* data, size_t size);
TriangleMesh load_mesh(psl::string_view filename);

void interpret_file(Context& context, psl::string_view filename);

void fileio_context(Context& ctx);

}  // namespace pine
