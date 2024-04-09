#pragma once
#include <psl/type_traits.h>
#include <psl/algorithm.h>
#include <psl/stdint.h>
#include <psl/check.h>

namespace psl {

template <typename T>
struct span {
  span() = default;
  span(T* ptr, size_t length) : ptr(ptr), length(length) {
  }
  template <RandomAccessIterator It>
  span(It first, It last) : ptr(&(*first)), length(size_t(last - first)) {
  }
  template <Range ARange>
  span(ARange&& range) : span(psl::begin(range), psl::end(range)) {
  }

  T* begin() {
    return ptr;
  }
  const T* begin() const {
    return ptr;
  }
  T* end() {
    return ptr + length;
  }
  const T* end() const {
    return ptr + length;
  }
  size_t size() const {
    return length;
  }
  T& operator[](size_t i) {
    psl_check(i < length);
    return ptr[i];
  }
  const T& operator[](size_t i) const {
    psl_check(i < length);
    return ptr[i];
  }

  span subspan(size_t start) const {
    psl_check(start <= length);
    return span(ptr + start, length - start);
  }
  span subspan(size_t start, size_t size) const {
    psl_check(start + size <= length);
    return span(ptr + start, size);
  }

private:
  T* ptr = nullptr;
  size_t length = 0;
};

}  // namespace psl
