#pragma once
#include <psl/utility.h>
#include <psl/memory.h>
#include <psl/check.h>

namespace psl {

struct nullopt_t {};

constexpr nullopt_t nullopt;

template <typename T>
struct optional {
  optional() = default;
  optional(nullopt_t){};
  ~optional() {
    reset();
  }

  template <typename U>
  requires psl::DifferentFrom<psl::Decay<U>, optional>
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
    this->reset();
    if (rhs) {
      psl::construct_at(ptr(), psl::move(rhs.value()));
      valid = psl::exchange(rhs.valid, false);
    }
    return *this;
  }

  optional& accept(optional rhs) {
    if (rhs) {
      this->reset();
      psl::construct_at(ptr(), psl::move(*rhs));
      valid = true;
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
    return ptr();
  }
  const T* operator->() const {
    return ptr();
  }

  void reset() {
    if (valid) {
      value().~T();
      valid = false;
    }
  }

  explicit operator bool() const {
    return valid;
  }

  friend bool operator==(const psl::optional<T>& lhs, const psl::optional<T>& rhs) {
    if (!lhs && !rhs)
      return true;
    else if (lhs && rhs)
      return *lhs == *rhs;
    else
      return false;
  }
  friend bool operator==(const T& lhs, const psl::optional<T>& rhs) {
    return rhs && lhs == *rhs;
  }
  friend bool operator==(const psl::optional<T>& lhs, const T& rhs) {
    return lhs && *lhs == rhs;
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
auto make_optional(T x) {
  return optional<T>(FWD(x));
}

template <typename Map>
auto find_or_nullopt(Map&& map, auto&& key) {
  if (auto it = map.find(key); it != map.end())
    return psl::make_optional(it->second);
  else
    return decltype(psl::make_optional(it->second))(psl::nullopt);
}

template <typename T>
psl::optional<T> fmap(psl::optional<T> x, auto f) {
  if (x)
    f(*x);
  return x;
}

}  // namespace psl
