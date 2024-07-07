#pragma once
#include <psl/memory.h>

namespace psl {

template <typename T, size_t max_static_bytes>
struct smart_unique_ptr {
  template <typename U>
  smart_unique_ptr(psl::unique_ptr<U> rhs) : ptr(psl::unique_ptr<T>(MOVE(rhs))), is_local(false) {
  }
  template <typename U>
  smart_unique_ptr(U x) : is_local(true) {
    static_assert(sizeof(U) <= max_static_bytes, "");
    psl::construct_at(local.template ptr<U>(), MOVE(x));
  }
  ~smart_unique_ptr() {
    if (is_local)
      psl::destruct_at(local.template ptr<T>());
  }
  smart_unique_ptr(smart_unique_ptr&& rhs) {
    if ((is_local = psl::exchange(rhs.is_local, false)))
      local = rhs.local;
    else
      ptr = MOVE(rhs.ptr);
  }
  smart_unique_ptr& operator=(smart_unique_ptr&& rhs) {
    if ((is_local = psl::exchange(rhs.is_local, false)))
      local = rhs.local;
    else
      ptr = MOVE(rhs.ptr);
    return *this;
  }

  T* get() {
    return is_local ? local.template ptr<T>() : ptr.get();
  }
  T* get() const {
    return is_local ? local.template ptr<T>() : ptr.get();
  }
  T& operator*() {
    return *get();
  }
  const T& operator*() const {
    return *get();
  }
  T* operator->() {
    return get();
  }
  T* operator->() const {
    return get();
  }

private:
  psl::unique_ptr<T> ptr;
  psl::Storage<max_static_bytes, 8> local;
  bool is_local;
};

}  // namespace psl