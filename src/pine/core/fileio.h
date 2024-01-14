#pragma once

#include <pine/core/geometry.h>
#include <pine/core/image.h>

#include <pine/psl/fstream.h>
#include <pine/psl/string.h>
#include <pine/psl/vector.h>
#include <pine/psl/memory.h>

namespace pine {

struct ScopedFile {
  ScopedFile(psl::string_view filename, psl::ios::OpenMode mode);

  template <typename T>
  T Read() {
    T val;
    Read(&val, sizeof(T));
    return val;
  }
  template <typename T>
  void Write(const T& val) {
    Write(&val, sizeof(T));
  }

  void Write(const void* data, size_t size);
  void Read(void* data, size_t size);

  size_t Size() const {
    return file.size();
  }
  bool Success() const {
    return file.is_open();
  }

  mutable psl::Fstream file;
  mutable size_t size = -1;
};

bool IsFileExist(psl::string_view filename);
psl::string GetFileDirectory(psl::string_view filename);
psl::string GetFileExtension(psl::string_view filename);
psl::string RemoveFileExtension(psl::string_view filename);
psl::string ChangeFileExtension(psl::string_view filename, psl::string ext);
psl::string AppendFileName(psl::string_view filename, psl::string content);

psl::string ReadStringFile(psl::string_view filename);

void WriteBinaryData(psl::string_view filename, const void* ptr, size_t size);
psl::vector<char> ReadBinaryData(psl::string_view filename);

psl::vector<uint8_t> ToUint8Image(vec2i size, int nchannel, const float* data);
void SaveImage(psl::string_view filename, vec2i size, int nchannel, const float* data);
void SaveImage(psl::string_view filename, vec2i size, int nchannel, const uint8_t* data);
Image read_image(psl::string_view filename);
Image read_image(void* data, size_t size);

TriangleMesh load_mesh(void* data, size_t size);
TriangleMesh load_mesh(psl::string_view filename);

void interpretFile(Context& context,psl::string_view filename);

template <typename... Ts>
void Serialize(psl::string_view filename, const Ts&... object) {
  auto data = Archive(object...);
  WriteBinaryData(filename, data.data(), data.size() * sizeof(data[0]));
}
template <typename... Ts>
auto Deserialize(psl::string_view filename) {
  return Unarchive<Ts...>(ReadBinaryData(filename));
}

}  // namespace pine
