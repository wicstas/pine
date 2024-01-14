#pragma once

#include <pine/psl/string.h>

namespace psl {

namespace ios {

enum OpenMode { in = 1 << 0, out = 1 << 1, binary = 1 << 2 };

inline OpenMode operator|(OpenMode a, OpenMode b) {
  return static_cast<OpenMode>(static_cast<int>(a) | static_cast<int>(b));
}

}  // namespace ios

class Fstream {
public:
  Fstream() = default;
  Fstream(psl::string_view filename, ios::OpenMode mode) {
    open(filename, mode);
  }
  ~Fstream() {
    close();
  }

  Fstream(const Fstream&) = delete;
  Fstream(Fstream&&) = delete;
  Fstream& operator=(const Fstream&) = delete;
  Fstream& operator=(Fstream&&) = delete;

  void open(psl::string_view filename, ios::OpenMode mode);

  void close();

  bool is_open() const;

  size_t size() const;

  void write(const void* data, size_t size);
  void read(void* data, size_t size) const;

private:
  void* file = nullptr;
  mutable size_t size_ = -1;
};

}  // namespace psl
