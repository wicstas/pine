#include <psl/fstream.h>

#include <stdio.h>

#include <filesystem>

namespace psl {

void Fstream::open(string_view filename, ios::OpenMode mode) {
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
    fclose((FILE *)file);
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
    size_t begin = ftell((FILE *)file);
    fseek((FILE *)file, 0, SEEK_END);
    size_t end = ftell((FILE *)file);
    fseek((FILE *)file, 0, SEEK_SET);
    size_ = end - begin;
  }

  return size_;
}

void Fstream::write(const void *data, size_t size) {
  fwrite(data, size, 1, (FILE *)file);
}
void Fstream::read(void *data, size_t size) const {
  size_t ret = fread(data, size, 1, (FILE *)file);
  (void)ret;
}

ScopedFile::ScopedFile(psl::string_view filename_view, psl::ios::OpenMode mode) {
  auto filename = (psl::string)filename_view;
  psl::replace(filename, '\\', '/');
  file.open(filename.c_str(), mode);
}
void ScopedFile::write(const void *data, size_t size) {
  if (file.is_open())
    file.write((char *)data, size);
}
void ScopedFile::read(void *data, size_t size) {
  if (file.is_open())
    file.read((char *)data, size);
}

}  // namespace psl