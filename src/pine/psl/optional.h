#pragma once

#include <pine/psl/utility.h>
#include <pine/psl/memory.h>

namespace psl {

struct nullopt_t {};

constexpr nullopt_t nullopt;

template <typename T>
struct optional {
  optional() = default;
  optional(nullopt_t){};
  ~optional() {
    if (valid) {
      value().~T();
      valid = false;
    }
  }

  template <typename U>
  requires psl::DifferentFrom<psl::DecayT<U>, optional>
  optional(U&& x) {
    psl::construct_at(ptr(), psl::forward<U>(x));
    valid = true;
  }

  optional(const optional& rhs) {
    if (rhs) {
      psl::construct_at(ptr(), rhs.value());
      valid = true;
    }
  }
  optional(optional&& rhs) {
    if (rhs) {
      psl::construct_at(ptr(), psl::move(rhs.value()));
      valid = psl::exchange(rhs.valid, false);
    }
  }
  optional& operator=(optional rhs) {
    this->~optional();
    if (rhs) {
      psl::construct_at(ptr(), psl::move(rhs.value()));
      valid = psl::exchange(rhs.valid, false);
    }
    return *this;
  }

  T& operator*() {
    return value();
  }
  const T& operator*() const {
    return value();
  }
  T* operator->() {
    return &value();
  }
  const T* operator->() const {
    return &value();
  }

  explicit operator bool() const {
    return valid;
  }

private:
  T& value() {
    psl_check(valid);
    return *ptr();
  }
  const T& value() const {
    psl_check(valid);
    return *ptr();
  }
  T* ptr() {
    return reinterpret_cast<T*>(storage);
  }
  const T* ptr() const {
    return reinterpret_cast<const T*>(storage);
  }

  alignas(T) unsigned char storage[sizeof(T)];
  bool valid = false;
};
template <typename T>
optional(T) -> optional<T>;

}  // namespace psl
