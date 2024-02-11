#pragma once

#include <atomic>

namespace pine {

template <typename T>
struct Local {
  Local() {}
  Local(const Local&) {}
  Local(Local&&) {}
  Local& operator=(Local) { return *this; }
  T& operator*() { return base; }
  const T& operator*() const { return base; }
  T* operator->() { return &base; }
  const T* operator->() const { return &base; }

 private:
  T base;
};

//TODO use this directly instead of Atomic ...[3]
struct SpinLock {
  void lock() {
    if (!flag->test_and_set(std::memory_order_acquire)) return;
    while (flag->test(std::memory_order_relaxed))
      ;
  }
  void unlock() { flag->clear(std::memory_order_release); }

 private:
  Local<std::atomic_flag> flag;
};

template <typename T>
struct Atomic {
  Atomic() = default;
  Atomic(T value) : value{value} {}
  Atomic& operator+=(T rhs) {
    lock.lock();
    value += rhs;
    lock.unlock();
    return *this;
  }
  operator T() const { return value; }

 private:
  T value;
  SpinLock lock;
};

}  // namespace pine
