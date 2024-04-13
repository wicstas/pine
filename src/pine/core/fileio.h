#pragma once
#include <pine/core/geometry.h>
#include <pine/core/image.h>

#include <psl/optional.h>
#include <psl/function.h>
#include <psl/fstream.h>
#include <psl/string.h>
#include <psl/vector.h>
#include <psl/memory.h>
#include <psl/span.h>
#include <psl/map.h>

namespace pine {

using Bytes = psl::vector<uint8_t>;
using BytesView = psl::span<uint8_t>;

psl::string read_string_file(psl::string_view filename);

void write_binary_file(psl::string_view filename, const void* ptr, size_t size);
Bytes read_binary_file(psl::string_view filename);

psl::map<psl::string, Bytes> read_folder(psl::string path);

Bytes serialize(const psl::map<psl::string, Bytes>& fs);
template <typename T>
T deserialize(BytesView bytes);

void save_image(psl::string filename, vec2i size, int nchannel, const float* data);
void save_image(psl::string filename, vec2i size, int nchannel, const uint8_t* data);

template <typename T>
Array2d<T> invert_y(const Array2d<T>& input) {
  auto output = Array2d<T>(input.size());
  for_2d(input.size(), [&](vec2i p) { output[p] = input[vec2i(p.x, input.size().y - 1 - p.y)]; });
  return output;
}
inline void save_image(psl::string filename, Array2d<vec3u8> pixels, bool should_invert_y = false) {
  if (should_invert_y)
    pixels = invert_y(pixels);
  save_image(filename, pixels.size(), 3, &pixels.data()[0][0]);
}
inline void save_image(psl::string filename, Array2d<vec4u8> pixels, bool should_invert_y = false) {
  if (should_invert_y)
    pixels = invert_y(pixels);
  save_image(filename, pixels.size(), 4, &pixels.data()[0][0]);
}
inline void save_image(psl::string filename, Array2d<vec3> pixels, bool should_invert_y = false) {
  if (should_invert_y)
    pixels = invert_y(pixels);
  save_image(filename, pixels.size(), 3, &pixels.data()[0][0]);
}
inline void save_image(psl::string filename, Array2d<vec4> pixels, bool should_invert_y = false) {
  if (should_invert_y)
    pixels = invert_y(pixels);
  save_image(filename, pixels.size(), 4, &pixels.data()[0][0]);
}
inline void save_image(psl::string filename, Image image, bool should_invert_y = false) {
  image.dispatch([&](auto&& x) { save_image(filename, x, should_invert_y); });
}

psl::optional<Image> image_from(void* data, size_t size);
psl::shared_ptr<Image> load_image(psl::string_view filename,
                                  psl::function<Bytes(psl::string_view)> reader);
inline psl::shared_ptr<Image> load_image(psl::string_view filename) {
  return load_image(filename, read_binary_file);
}

void scene_from(Scene& scene, void* tiny_gltf_model, mat4 m);
UberMaterial material_from(void* tiny_gltf_model);

void scene_from(Scene& scene, psl::string file_name, const psl::map<psl::string, Bytes>& fs, mat4 m);
UberMaterial material_from(psl::string file_name, const psl::map<psl::string, Bytes>& fs);

void interpret_file(Context& context, psl::string_view filename);

void fileio_context(Context& ctx);

}  // namespace pine
