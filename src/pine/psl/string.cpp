#include <pine/psl/string.h>

namespace psl {

size_t strlen(const char* str) {
  if (str == nullptr)
    return 0;
  size_t len = 0;
  while (*(str++))
    ++len;
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

string::string() {
  reserve(1);
}
string::string(const char* cstr) {
  resize(psl::strlen(cstr));
  psl::copy(begin(), range(cstr, size()));
}

string::string(const char* cstr, size_t len) {
  resize(len);
  psl::copy(begin(), range(cstr, size()));
}

string string::substr(size_t pos, size_t len) const {
  pos = clamp(pos, size_t{0}, size());
  if (len == size_t(-1))
    return string(data() + pos, size() - pos);
  else
    return string(data() + pos, min(len, size() - pos));
}

string& string::operator=(const char* str) {
  resize(psl::strlen(str));
  psl::copy(begin(), range(str, str + size()));
  return *this;
}
string_view string::subview(size_t pos, size_t len) const {
  return string_view(*this).substr(pos, len);
}

string& string::operator=(string_view str) {
  resize(psl::size(str));
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
string& string::operator+=(const char* rhs) {
  return (*this) += string_view(rhs);
}
string& string::operator+=(char c) {
  push_back(c);
  return *this;
}
string operator+(string lhs, string_view rhs) {
  return lhs += rhs;
}

int stoi(string_view str) {
  int number = 0;
  bool is_neg = false;
  for (size_t j = 0; j < psl::size(str); j++) {
    if (str[j] == '.')
      break;
    if (j == 0 && str[j] == '-')
      is_neg = true;
    else
      number = number * 10 + str[j] - '0';
  }
  return is_neg ? -number : number;
}

int64_t stoi64(string_view str) {
  int64_t number = 0;
  bool is_neg = false;
  for (size_t j = 0; j < psl::size(str); j++) {
    if (str[j] == '.')
      break;
    if (j == 0 && str[j] == '-')
      is_neg = true;
    else
      number = number * 10 + str[j] - '0';
  }
  return is_neg ? -number : number;
}

float stof(string_view str) {
  float number = 0.0f;
  bool is_neg = false;
  bool pass_decimal_point = false;
  float scale = 0.1f;
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

string space_by(vector<string> input, string spacer) {
  auto result = string{};
  for (string& c : input)
    result += c + (&c == &input.back() ? "" : spacer);
  return result;
}
string space_by(string input, string spacer) {
  auto result = string{};
  for (char& c : input)
    result += psl::to_string(c) + (&c == &input.back() ? "" : spacer);
  return result;
}

}  // namespace psl