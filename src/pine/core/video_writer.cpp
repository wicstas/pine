#include <pine/core/video_writer.h>
#include <pine/core/fileio.h>

#include <contrib/libgwavi/gwavi.h>
#include <contrib/stb_image_write.h>

namespace pine {

VideoWriter::VideoWriter(psl::string filename, vec2i size, int fps)
    : filename(filename), size(size){
  ptr = psl::opaque_shared_ptr(gwavi_open(filename.c_str(), size.x, size.y, "JPEG", fps, nullptr),
                               [=](gwavi_t* ptr) {
                                 if (gwavi_close(ptr))
                                   Warning("Unable to close `", filename, '`');
                               });
  if (!ptr)
    Fatal("Unable to open `", filename, '`');
}
void VideoWriter::add_frame(const Array2d3f& pixels) {
  auto pixel_data = to_uint8_array(size, 3, (float*)pixels.data(), true);
  psl::vector<uint8_t> bytes;
  stbi_write_jpg_to_func(
      +[](void* context, void* data, int size) {
        auto& bytes = *(psl::vector<uint8_t>*)context;
        auto ptr = (uint8_t*)data;
        auto pos = bytes.size();
        bytes.resize(bytes.size() + size);
        for (int i = 0; i < size; i++)
          bytes[pos + i] = ptr[i];
      },
      &bytes, pixels.width(), pixels.height(), 3, pixel_data.data(), 90);
  if (gwavi_add_frame((gwavi_t*)ptr.get(), bytes.data(), bytes.byte_size()))
    Warning("Failed to add frame to `", filename, '`');
}
void VideoWriter::done() {
  ptr.reset();
}

}  // namespace pine
