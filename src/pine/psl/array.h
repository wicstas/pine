#pragma once

#include <pine/psl/algorithm.h>

#include <initializer_list>

namespace psl {

template <typename T, size_t N>
struct Array {
  using Iterator = T*;
  using ConstIterator = const T*;

  Array() = default;
  template <typename... Ts>
  requires same_type<T, Ts...>
  Array(Ts... xs) : data{xs...} {};

  T& operator[](size_t i) {
    return data[i];
  }
  const T& operator[](size_t i) const {
    return data[i];
  }

  Iterator begin() {
    return data;
  }
  Iterator end() {
    return data + size();
  }
  ConstIterator begin() const {
    return data;
  }
  ConstIterator end() const {
    return data + size();
  }
  size_t size() const {
    return N;
  }
  T* ptr() {
    return data;
  }
  const T* ptr() const {
    return data;
  }

  template <typename U, size_t M>
  requires EqualityComparable<T, U>
  bool operator==(const Array<U, M>& rhs) const {
    if constexpr (N != M)
      return false;
    for (size_t i = 0; i < N; i++)
      if (data[i] != rhs[i])
        return false;
    return true;
  }
  template <typename U, size_t M>
  requires EqualityComparable<T, U>
  bool operator!=(const Array<U, M>& rhs) const {
    return !((*this) == rhs);
  }

private:
  T data[N]{};
};
template <typename T, typename... Ts>
requires same_type<T, Ts...>
Array(T, Ts...) -> Array<T, 1 + sizeof...(Ts)>;

}  // namespace psl