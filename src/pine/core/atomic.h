#pragma once

#include <psl/type_traits.h>

#include <atomic>

namespace pine {

template <typename T>
struct Local {
  Local() {
  }
  Local(const Local&) {
  }
  Local(Local&&) {
  }
  Local& operator=(Local) {
    return *this;
  }
  T& operator*() {
    return base;
  }
  const T& operator*() const {
    return base;
  }
  T* operator->() {
    return &base;
  }
  const T* operator->() const {
    return &base;
  }

private:
  T base;
};

struct SpinLock {
  void lock() {
    while (flag->test_and_set(std::memory_order_acquire))
      ;
  }
  void unlock() {
    flag->clear(std::memory_order_release);
  }

private:
  Local<std::atomic_flag> flag;
};

template <typename T>
struct Atomic {
  Atomic() = default;
  Atomic(T value) : value{value} {
  }
  Atomic& operator+=(T rhs) {
    lock.lock();
    value += rhs;
    lock.unlock();
    return *this;
  }
  Atomic& operator-=(T rhs) {
    lock.lock();
    value -= rhs;
    lock.unlock();
    return *this;
  }
  Atomic& operator*=(T rhs) {
    lock.lock();
    value *= rhs;
    lock.unlock();
    return *this;
  }
  Atomic& operator/=(T rhs) {
    lock.lock();
    value /= rhs;
    lock.unlock();
    return *this;
  }
  operator T() const {
    lock.lock();
    auto val = value;
    lock.unlock();
    return val;
  }

private:
  T value;
  mutable SpinLock lock;
};

template <psl::FloatingPoint T>
struct Atomic<T> {
  Atomic() = default;
  Atomic(T v) : value(v){};

  Atomic(const Atomic& rhs) : value(rhs.value.load(std::memory_order_acquire)) {
  }
  Atomic(Atomic&& rhs) : value(rhs.value.load(std::memory_order_acquire)) {
  }
  Atomic& operator=(const Atomic& rhs) {
    value = rhs.value.load(std::memory_order_acquire);
    return *this;
  }
  Atomic& operator=(Atomic&& rhs) {
    value = rhs.value.load(std::memory_order_acquire);
    return *this;
  }

  operator T() const {
    return value.load(std::memory_order_relaxed);
  }
  void operator=(T rhs) {
    value = rhs;
  }
  void operator+=(T rhs) {
    auto old = value.load(std::memory_order_consume);
    auto desired = old + rhs;
    while (!value.compare_exchange_weak(old, desired))
      desired = old + rhs;
  }
  void operator-=(T rhs) {
    auto old = value.load(std::memory_order_consume);
    auto desired = old - rhs;
    while (!value.compare_exchange_weak(old, desired))
      desired = old - rhs;
  }

private:
  std::atomic<T> value;
};
template <psl::Integral T>
struct Atomic<T> {
  Atomic() = default;
  Atomic(T v) : value(v){};

  Atomic(const Atomic& rhs) : value(rhs.value.load(std::memory_order_acquire)) {
  }
  Atomic(Atomic&& rhs) : value(rhs.value.load(std::memory_order_acquire)) {
  }
  Atomic& operator=(const Atomic& rhs) {
    value = rhs.value.load(std::memory_order_acquire);
    return *this;
  }
  Atomic& operator=(Atomic&& rhs) {
    value = rhs.value.load(std::memory_order_acquire);
    return *this;
  }

  operator T() const {
    return value.load(std::memory_order_relaxed);
  }
  auto& operator=(T rhs) {
    value = rhs;
    return *this;
  }
  void operator+=(T rhs) {
    value += rhs;
  }
  void operator-=(T rhs) {
    value -= rhs;
  }
  T operator++(int) {
    return value++;
  }
  T operator--(int) {
    return value--;
  }

private:
  std::atomic<T> value;
};

template <typename T>
psl::string to_string(Atomic<T> val) {
  using psl::to_string;
  return to_string(T(val));
}

}  // namespace pine
