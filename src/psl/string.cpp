#include <psl/string.h>

namespace psl {

size_t strlen(const char* str) {
  if (str == nullptr) return 0;
  size_t len = 0;
  while (*(str++)) ++len;
  return len;
}

int strcmp(const char* lhs, const char* rhs) {
  if (!lhs && !rhs)
    return 0;
  else if (!lhs && rhs)
    return -1;
  else if (lhs && !rhs)
    return 1;

  for (size_t i = 0;; ++i) {
    if (!lhs[i] && !rhs[i])
      return 0;
    else if (!lhs[i] && rhs[i])
      return -1;
    else if (lhs[i] && !rhs[i])
      return 1;
    else if (lhs[i] < rhs[i])
      return -1;
    else if (lhs[i] > rhs[i])
      return 1;
  }
}

string::string() { resize(0); }
string::string(size_t len) { resize(len); }
string::string(const char* cstr) {
  resize(psl::strlen(cstr));
  psl::copy(begin(), range(cstr, size()));
}

string::string(const char* cstr, size_t len) {
  resize(len);
  psl::copy(begin(), range(cstr, size()));
}
string::string(const string& rhs) : base() {
  resize(rhs.size());
  psl::copy(begin(), rhs);
}
string& string::operator=(string rhs) {
  psl::swap(ptr, rhs.ptr);
  psl::swap(len, rhs.len);
  psl::swap(reserved, rhs.reserved);
  psl::swap(allocator, rhs.allocator);
  return *this;
}
void string::resize(size_t len) {
  reserve(len + 1);
  base::resize(len);
  data()[len] = '\0';
}
string string::substr(size_t pos, size_t len) const {
  pos = clamp(pos, size_t{0}, size());
  if (len == size_t(-1))
    return string(data() + pos, size() - pos);
  else
    return string(data() + pos, min(len, size() - pos));
}

string& string::operator=(const char* cstr) {
  resize(psl::strlen(cstr));
  psl::copy(begin(), range(cstr, size()));
  return *this;
}
string_view string::subview(size_t pos, size_t len) const {
  return string_view(*this).subview(pos, len);
}
string_view string::subview(Iterator p0, Iterator p1) const {
  return subview(p0 - begin(), p1 - p0);
}
string_view string::subview(Iterator p0) const { return subview(p0 - begin(), end() - p0); }

string& string::operator=(string_view str) {
  resize(str.size());
  psl::copy(begin(), str);
  return *this;
}

string& string::operator+=(const string& rhs) {
  auto old_len = size();

  resize(size() + psl::size(rhs));
  psl::copy(begin() + old_len, rhs);
  return *this;
}
string& string::operator+=(class string_view rhs) {
  auto old_len = size();

  resize(size() + psl::size(rhs));
  psl::copy(begin() + old_len, rhs);
  return *this;
}
string& string::operator+=(const char* rhs) { return (*this) += string_view(rhs); }
string& string::operator+=(char c) {
  push_back(c);
  return *this;
}

string_view string_view::subview(size_t pos, iterator end) const {
  return subview(pos, distance(begin(), end) - pos);
}
string_view string_view::subview(size_t pos, size_t len) const {
  pos = clamp(pos, size_t{0}, size());
  if (len == size_t(-1))
    return string_view(data() + pos, size() - pos);
  else
    return string_view(data() + pos, min(len, size() - pos));
}

string to_string(float x) {
  if (psl::isnan(x)) return "nan";
  if (x > psl::numeric_limits<float>::max()) return "inf";
  if (x < -psl::numeric_limits<float>::max()) return "-inf";

  auto neg = x < 0;
  auto str = psl::string(neg ? "-" : "");
  x = psl::abs(x);
  str += psl::to_string(static_cast<int64_t>(x)) + ".";
  x = psl::fract(x);

  for (int i = 0; i < 4; ++i) {
    x *= 10;
    str.push_back('0' + static_cast<char>(x));
    x = psl::absfract(x);
  }

  return str;
}
string to_string(double x) {
  if (psl::isnan(x)) return "nan";
  if (x > psl::numeric_limits<double>::max()) return "inf";
  if (x < -psl::numeric_limits<double>::max()) return "-inf";

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

int stoi(string_view str) {
  auto number = 0;
  auto is_neg = false;
  for (size_t j = 0; j < psl::size(str); j++) {
    if (str[j] == '.') break;
    if (j == 0 && str[j] == '-')
      is_neg = true;
    else
      number = number * 10 + str[j] - '0';
  }
  return is_neg ? -number : number;
}

int64_t stoi64(string_view str) {
  auto number = 0;
  auto is_neg = false;
  for (size_t j = 0; j < psl::size(str); j++) {
    if (str[j] == '.') break;
    if (j == 0 && str[j] == '-')
      is_neg = true;
    else
      number = number * 10 + str[j] - '0';
  }
  return is_neg ? -number : number;
}

float stof(string_view str) {
  auto number = 0.0f;
  auto is_neg = false;
  auto pass_decimal_point = false;
  auto scale = 0.1f;
  for (size_t j = 0; j < psl::size(str); j++) {
    if (j == 0 && str[j] == '-')
      is_neg = true;
    else if (!pass_decimal_point && str[j] == '.')
      pass_decimal_point = true;
    else if (!pass_decimal_point)
      number = number * 10 + str[j] - '0';
    else {
      number += (str[j] - '0') * scale;
      scale *= 0.1f;
    }
  }
  return is_neg ? -number : number;
}

}  // namespace psl