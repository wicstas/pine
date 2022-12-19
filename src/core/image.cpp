#include <core/image.hpp>

#include <fstream>

namespace pine {

void writeImageAsTGA(std::string fileName, const ImageLDR& image) {
  auto file = std::ofstream(fileName, std::ios::binary);
  auto reordered = image;
  for (int i = 0; i < image.nPixels(); ++i)
    reordered.data()[i] = shuffle(image.data()[i], vec3i(2, 1, 0));

  const char headers[]{
      0,  // ID Length
      0,  // Color Map Type
      2,  // Image Type
      0,
      0,
      0,
      0,
      0,  // Color Map Specification
      0,
      0,
      0,
      0,
      char(image.width()),
      char(image.width() >> 8),
      char(image.height()),
      char(image.height() >> 8),
      24,
      0b0001000  // Image Specification
  };
  file.write(headers, sizeof(headers));
  file.write((char*)(reordered.data()), 3 * image.nPixels());
}

}  // namespace pine
