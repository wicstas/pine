#pragma once

#include <psl/type_traits.h>
#include <psl/optional.h>
#include <psl/vector.h>
#include <psl/limits.h>
#include <psl/math.h>

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
    auto ptr = ::new T[size]{};
    return ptr;
  }
  void free(T* ptr) const {
    ::delete[] ptr;
  }

  void construct_at(T* ptr) const {
    *ptr = ' ';
  }
  void construct_at(T* ptr, T c) const {
    *ptr = c;
  }

  void destruct_at(T* ptr) const {
    *ptr = 0;
  }
};

class string : public vector<char, string_allocator<char>> {
public:
  using base = vector<char, string_allocator<char>>;

  string();
  explicit string(size_t len);
  template <Range ARange>
  explicit string(ARange&& range) : string(psl::size(range)) {
    psl::copy(begin(), range);
  }
  template <ForwardIterator It>
  string(It first, It last) : string(psl::range(first, last)) {
  }
  string(const char* cstr);
  string(const char* cstr, size_t len);

  string(string&& rhs) = default;
  string(const string& rhs);

  string& operator=(string rhs);

  void resize(size_t len);

  string substr(size_t pos, size_t len = -1) const;
  class string_view subview(size_t pos, size_t len = -1) const;

  string& operator=(const char* str);
  string& operator=(class string_view str);

  string& operator+=(const string& rhs);
  string& operator+=(class string_view rhs);
  string& operator+=(const char* rhs);
  string& operator+=(char c);

  friend string operator+(string lhs, const char* rhs) {
    return lhs += rhs;
  }
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
    return !(lhs == rhs);
  }
  friend bool operator<(const string& lhs, const string& rhs) {
    return strcmp(lhs.c_str(), rhs.c_str()) < 0;
  }
  friend bool operator>(const string& lhs, const string& rhs) {
    return strcmp(lhs.c_str(), rhs.c_str()) > 0;
  }
  friend bool operator==(const string& lhs, const char* rhs) {
    return strcmp(lhs.c_str(), rhs) == 0;
  }
  friend bool operator!=(const string& lhs, const char* rhs) {
    return !(lhs == rhs);
  }
  friend bool operator<(const string& lhs, const char* rhs) {
    return strcmp(lhs.c_str(), rhs) < 0;
  }
  friend bool operator>(const string& lhs, const char* rhs) {
    return strcmp(lhs.c_str(), rhs) > 0;
  }
  friend bool operator==(const char* lhs, const string& rhs) {
    return strcmp(lhs, rhs.c_str()) == 0;
  }
  friend bool operator!=(const char* lhs, const string& rhs) {
    return !(lhs == rhs);
  }
  friend bool operator<(const char* lhs, const string& rhs) {
    return strcmp(lhs, rhs.c_str()) < 0;
  }
  friend bool operator>(const char* lhs, const string& rhs) {
    return strcmp(lhs, rhs.c_str()) > 0;
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
  string_view(Range auto&& range) : string_view(psl::begin(range), psl::end(range)) {
  }
  string_view substr(size_t pos, iterator end) const;
  string_view substr(size_t pos, size_t len = -1) const;

  friend string operator+(string_view lhs, const string& rhs) {
    return string(lhs) + rhs;
  }
  friend string operator+(string lhs, string_view rhs) {
    return lhs += rhs;
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

  const char& front() const {
    psl_check(size() != 0);
    return str[0];
  }
  const char& back() const {
    psl_check(size() != 0);
    return str[size() - 1];
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

  friend bool operator==(string_view lhs, string_view rhs) {
    if (lhs.size() != rhs.size())
      return false;
    for (size_t i = 0; i < lhs.size(); i++)
      if (lhs[i] != rhs[i])
        return false;
    return true;
  }
  friend bool operator!=(string_view lhs, string_view rhs) {
    return !(lhs == rhs);
  }
  friend bool operator<(string_view lhs, string_view rhs) {
    for (size_t i = 0; i < rhs.size(); i++) {
      if (i == lhs.size())
        return true;
      if (lhs[i] < rhs[i])
        return true;
      else if (lhs[i] > rhs[i])
        return false;
    }
    return false;
  }
  friend bool operator>(string_view lhs, string_view rhs) {
    for (size_t i = 0; i < lhs.size(); i++) {
      if (i == rhs.size())
        return true;
      if (lhs[i] > rhs[i])
        return true;
      else if (lhs[i] < rhs[i])
        return false;
    }
    return false;
  }

  const char* str = nullptr;
  size_t len = 0;

  inline static const size_t npos = (size_t)-1;
};

inline string string_n_of(size_t n, char x) {
  auto str = string(n);
  psl::fill(str, x);
  return str;
}

template <typename T>
string to_string_forward(const T& x);

inline string to_string(const char* x) {
  return string(x);
}
inline string to_string(string x) {
  return x;
}
inline string to_string(string_view x) {
  return string(x);
}
// Not doing `to_string(bool x)` for overload resolution reasons
template <SameAs<bool> T>
string to_string(T x) {
  return x ? "true" : "false";
}
// Not doing `to_string(char x)` for overload resolution reasons
template <SameAs<char> T>
string to_string(T x) {
  return string_n_of(1, x);
}
template <Integral T>
string to_string(T x) {
  constexpr auto max_len = 16;
  char str[max_len] = {};
  auto i = max_len;

  auto negative = x < 0;
  x = psl::abs(x);
  do {
    str[--i] = '0' + x % 10;
    x /= 10;
  } while (x && i > 1);

  if (negative)
    str[--i] = '-';

  return string(str + i, max_len - i);
}

string to_string(float x);
string to_string(double x);

template <typename T, typename U>
string to_string(pair<T, U> x) {
  return "{" + to_string_forward(x.first) + ", " + to_string_forward(x.second) + "}";
}
template <typename T, size_t N>
string to_string(T (&&range)[N]) {
  auto r = string{"["};
  for (auto&& x : range)
    r += to_string_forward(x) + " ";
  if (r.back() == ' ')
    r.pop_back();
  r.push_back(']');
  return r;
}
template <Range ARange>
string to_string(ARange&& range) {
  auto r = string{"["};
  for (auto&& x : range)
    r += to_string_forward(x) + " ";
  if (r.back() == ' ')
    r.pop_back();
  r.push_back(']');
  return r;
}

template <typename T>
string to_string(Ref<T> x) {
  return "ref{" + to_string_forward(x) + "}";
}
template <typename T>
string to_string(T* x) {
  return "*{" + to_string_forward(*x) + "}";
}
template <typename T>
string to_string(const psl::optional<T>& x) {
  return x ? to_string_forward(*x) : "";
}
template <typename T>
requires requires(const T& x) { x.to_string(); }
string to_string(const T& x) {
  return x.to_string();
}
template <typename T>
string to_string_forward(const T& x) {
  return to_string(x);
}

template <typename... Ts>
requires(sizeof...(Ts) > 1)
string to_string(Ts&&... xs) {
  return (to_string(psl::forward<Ts>(xs)) + ...);
}

int stoi(string_view str);
int64_t stoi64(string_view str);
float stof(string_view str);

string space_by(vector<string> input, string spacer);
string space_by(string input, string spacer);

string from_last_of(const string& str, char c);

}  // namespace psl
