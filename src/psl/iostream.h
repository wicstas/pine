#pragma once

#include <psl/string.h>

namespace psl {

class ostream {
public:
  template <typename T>
  friend ostream& operator<<(ostream& os, T&& val) {
    string str = psl::to_string(psl::forward<T>(val));
    write(str.c_str(), psl::size(str) * sizeof(str[0]));
    return os;
  }

private:
  static void write(const char* str, size_t size);
};

extern ostream cout;
constexpr char endl = '\n';

}  // namespace psl
