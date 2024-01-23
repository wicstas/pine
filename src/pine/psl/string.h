#pragma once

#include <pine/psl/type_traits.h>
#include <pine/psl/vector.h>
#include <pine/psl/limits.h>
#include <pine/psl/math.h>

namespace psl {

inline bool isdigit(char c) {
  return c >= '0' && c <= '9';
}
inline bool isalpha(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}
inline bool isspace(char c) {
  return c == ' ' || c == '\t';
}
inline bool isnewline(char c) {
  return c == '\n' || c == '\r' || c == '\f';
}

size_t strlen(const char* str);
int strcmp(const char* lhs, const char* rhs);

template <typename T>
struct string_allocator {
  T* alloc(size_t size) const {
    return ::new T[size + 1]{};
  }
  void free(T* ptr) const {
    ::delete[] ptr;
  }

  void construct_at(T* ptr) const {
    *ptr = 0;
  }
  void construct_at(T* ptr, T c) const {
    *ptr = c;
  }

  void destruct_at(T* ptr) const {
    *ptr = 0;
  }
};

class string : public VectorBase<char, string_allocator<char>> {
public:
  using base = VectorBase<char, string_allocator<char>>;
  using base::base;

  string();
  string(const char* cstr);
  string(const char* cstr, size_t len);
  string substr(size_t pos, size_t len = -1) const;
  class string_view subview(size_t pos, size_t len = -1) const;

  string& operator=(const char* str);
  string& operator=(class string_view str);

  string& operator+=(const string& rhs);
  string& operator+=(class string_view rhs);
  string& operator+=(const char* rhs);
  string& operator+=(char c);

  friend string operator+(string lhs, const string& rhs) {
    return lhs += rhs;
  }
  friend string operator+(string lhs, char c) {
    return lhs += c;
  }

  const char* c_str() const {
    return data();
  }

  friend bool operator==(const string& lhs, const string& rhs) {
    return strcmp(lhs.c_str(), rhs.c_str()) == 0;
  }
  friend bool operator!=(const string& lhs, const string& rhs) {
    return strcmp(lhs.c_str(), rhs.c_str()) != 0;
  }
  friend bool operator<(const string& lhs, const string& rhs) {
    return strcmp(lhs.c_str(), rhs.c_str()) < 0;
  }
  friend bool operator>(const string& lhs, const string& rhs) {
    return strcmp(lhs.c_str(), rhs.c_str()) > 0;
  }

  inline static const size_t npos = static_cast<size_t>(-1);
};

class string_view {
public:
  using iterator = const char*;

  string_view() = default;
  string_view(const char* first, const char* last) : str(first), len(last - first) {
  }
  string_view(const char* str) : str(str), len(psl::strlen(str)) {
  }
  string_view(const char* str, size_t len) : str(str), len(len) {
  }
  string_view(const string& str) : str(str.c_str()), len(psl::size(str)) {
  }
  string_view substr(size_t pos, iterator end) const {
    return substr(pos, distance(begin(), end) - pos);
  }
  string_view substr(size_t pos, size_t len = -1) const {
    pos = clamp(pos, size_t{0}, size());
    if (len == size_t(-1))
      return string_view(data() + pos, size() - pos);
    else
      return string_view(data() + pos, min(len, size() - pos));
  }

  friend string operator+(string_view lhs, const string& rhs) {
    return string(lhs) + rhs;
  }
  friend string operator+(const string& lhs, string_view rhs) {
    return lhs + string(rhs);
  }
  friend string operator+(string_view lhs, const char* rhs) {
    return string(lhs) + rhs;
  }
  friend string operator+(const char* lhs, string_view rhs) {
    return lhs + string(rhs);
  }

  explicit operator string() const {
    return string{str, len};
  }

  const char& operator[](size_t i) const {
    return str[i];
  }

  const char* begin() const {
    return str;
  }
  const char* end() const {
    return str + size();
  }

  const char* data() const {
    return str;
  }
  const char* c_str() const {
    return str;
  }

  size_t size() const {
    return len;
  }

  friend bool operator==(const string_view& lhs, const string_view& rhs) {
    if (lhs.size() != rhs.size())
      return false;
    for (size_t i = 0; i < lhs.size(); i++)
      if (lhs[i] != rhs[i])
        return false;
    return true;
  }
  friend bool operator!=(const string_view& lhs, const string_view& rhs) {
    return !(lhs == rhs);
  }
  friend bool operator<(const string_view& lhs, const string_view& rhs) {
    return strcmp(lhs.c_str(), rhs.c_str()) < 0;
  }
  friend bool operator>(const string_view& lhs, const string_view& rhs) {
    return strcmp(lhs.c_str(), rhs.c_str()) > 0;
  }

  const char* str = nullptr;
  size_t len = 0;

  inline static const size_t npos = (size_t)-1;
};

inline string to_string(const char* x) {
  return string{x};
}
inline string to_string(string x) {
  return x;
}
inline string to_string(string_view x) {
  return string{x};
}
template <SameAs<bool> T>
string to_string(T x) {
  return x ? "true" : "false";
}
template <SameAs<char> T>
string to_string(T x) {
  return string{1, x};
}
template <Integral T>
string to_string(T x) {
  constexpr int max_len = 16;
  char str[max_len] = {};
  int i = max_len;

  bool negative = x < 0;
  x = psl::abs(x);
  do {
    str[--i] = '0' + x % 10;
    x /= 10;
  } while (x && i > 1);

  if (negative)
    str[--i] = '-';

  return string(str + i, max_len - i);
}

template <FloatingPoint T>
string to_string(T x) {
  if (psl::isnan(x))
    return "nan";
  if (x > psl::numeric_limits<float>::max())
    return "inf";
  if (x < -psl::numeric_limits<float>::max())
    return "-inf";

  auto neg = x < 0;
  auto str = psl::string(neg ? "-" : "");
  x = psl::abs(x);
  str += psl::to_string(static_cast<int64_t>(x)) + ".";
  x = psl::fract(x);

  for (int i = 0; i < 8; ++i) {
    x *= 10;
    str.push_back('0' + static_cast<char>(x));
    x = psl::absfract(x);
  }

  return str;
}
template <typename T, typename U>
string to_string(pair<T, U> x) {
  return "{" + to_string(x.first) + ", " + to_string(x.second) + "}";
}
template <Range ARange>
string to_string(ARange&& range) {
  auto r = string{"["};
  for (auto& x : range)
    r += to_string(x) + " ";
  if (psl::size(range))
    r.pop_back();
  r.push_back(']');
  return r;
}

template <typename T>
string to_string(ref<T> x) {
  return "ref{" + to_string(x) + "}";
}

template <typename... Ts>
requires(sizeof...(Ts) > 1)
string to_string(Ts&&... xs) {
  return (to_string(psl::forward<Ts>(xs)) + ...);
}

int stoi(string_view str);
float stof(string_view str);

string space_by(vector<string> input, string spacer);
string space_by(string input, string spacer);

}  // namespace psl
