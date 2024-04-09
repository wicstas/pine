#pragma once
#include <pine/core/array.h>

#include <psl/function.h>

namespace pine {

struct VideoWriter {
  VideoWriter(psl::string filename, vec2i size, int fps);
  void add_frame(const Array2d3f& pixels);
  void add_frame(const Array2d4f& pixels) {
    add_frame(Array2d3f::from(pixels));
  }
  void done();

private:
  psl::string filename;
  vec2i size;
  psl::opaque_shared_ptr ptr;
};

}  // namespace pine
