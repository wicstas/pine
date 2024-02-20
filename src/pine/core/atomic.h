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

  Atomic(const Atomic& rhs) : value(rhs.value.load(std::memory_order_relaxed)) {
  }
  Atomic(Atomic&& rhs) : value(rhs.value.load(std::memory_order_relaxed)) {
  }
  Atomic& operator=(const Atomic& rhs) {
    value = rhs.value.load(std::memory_order_relaxed);
    return *this;
  }
  Atomic& operator==(Atomic&& rhs) {
    value = rhs.value.load(std::memory_order_relaxed);
    return *this;
  }

  operator T() const {
    return value;
  }
  void operator=(T rhs) {
    value = rhs;
  }
  void operator+=(T rhs) {
    value += rhs;
  }

private:
  std::atomic<T> value;
};
template <psl::Integral T>
struct Atomic<T> {
  Atomic() = default;
  Atomic(T v) : value(v){};

  Atomic(const Atomic& rhs) : value(rhs.value.load(std::memory_order_relaxed)) {
  }
  Atomic(Atomic&& rhs) : value(rhs.value.load(std::memory_order_relaxed)) {
  }
  Atomic& operator=(const Atomic& rhs) {
    value = rhs.value.load(std::memory_order_relaxed);
    return *this;
  }
  Atomic& operator==(Atomic&& rhs) {
    value = rhs.value.load(std::memory_order_relaxed);
    return *this;
  }

  operator T() const {
    return value;
  }
  auto& operator=(T rhs) {
    value = rhs;
    return *this;
  }
  void operator+=(T rhs) {
    value += rhs;
  }
  T operator++(int) {
    return value++;
  }

private:
  std::atomic<T> value;
};

}  // namespace pine
