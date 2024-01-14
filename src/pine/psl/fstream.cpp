#include <pine/psl/fstream.h>

#include <stdio.h>

#include <filesystem>

namespace psl {

// void create_directory(string_view dir) {
//   std::filesystem::create_directory(dir.c_str());
// }

// string get_directory(string filename) {
//   psl::replace(filename, '\\', '/');

//   auto forwardslash = psl::find_last_of(filename, '/') - begin(filename);
//   if (psl::find_last_of(filename, '/') != end(filename))
//     filename = trim(filename, 0, forwardslash);

//   return filename + "/";
// }

void Fstream::open(string_view filename, ios::OpenMode mode) {
  // create_directory(get_directory((string)filename));

  close();

  if (mode & ios::in)
    file = fopen(filename.data(), mode & ios::binary ? "rb" : "r");
  else if (mode & ios::out)
    file = fopen(filename.data(), mode & ios::binary ? "wb" : "w");
  else
    file = fopen(filename.data(), mode & ios::binary ? "w+b" : "w+");
}

void Fstream::Fstream::close() {
  if (file) {
    fclose((FILE*)file);
    file = nullptr;
  }
}

bool Fstream::is_open() const {
  return file != nullptr;
}

size_t Fstream::size() const {
  if (!file)
    return 0;
  if (size_ == size_t(-1)) {
    size_t begin = ftell((FILE*)file);
    fseek((FILE*)file, 0, SEEK_END);
    size_t end = ftell((FILE*)file);
    fseek((FILE*)file, 0, SEEK_SET);
    size_ = end - begin;
  }

  return size_;
}

void Fstream::write(const void* data, size_t size) {
  fwrite(data, size, 1, (FILE*)file);
}
void Fstream::read(void* data, size_t size) const {
  size_t ret = fread(data, size, 1, (FILE*)file);
  (void)ret;
}

}  // namespace psl