#pragma once

#include <atomic>

namespace pine {

template <typename T>
struct Atomic {
  Atomic() = default;
  Atomic(T value) : base{value} {
  }
  Atomic(const Atomic& rhs) {
    base.store(rhs.base.load());
  }
  Atomic(Atomic&& rhs) {
    base.store(rhs.base.load());
  }
  Atomic& operator=(Atomic rhs) {
    base.store(rhs.base.load());
    return *this;
  }

  Atomic& operator=(T value) {
    base.store(value);
    return *this;
  }
  Atomic& operator+=(T rhs) {
    base += rhs;
    return *this;
  }
  operator T() const {
    return base.load();
  }

private:
  std::atomic<T> base;
};

template <typename T>
struct Heavy {
  Heavy() {
  }
  Heavy(const Heavy&) {
  }
  Heavy(Heavy&&) {
  }
  Heavy& operator=(Heavy) {
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
    if (!flag->test_and_set(std::memory_order_acquire))
      return;
    while (flag->test(std::memory_order_relaxed))
      ;
  }
  void unlock() {
    flag->clear(std::memory_order_release);
  }

private:
  Heavy<std::atomic_flag> flag;
};

}  // namespace pine
