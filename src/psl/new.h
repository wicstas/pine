#pragma once
#include <new>

namespace psl {

template <typename T, typename... Args>
void construct_at(T* ptr, Args&&... args) {
  ::new (ptr) T(psl::forward<Args>(args)...);
}

template <typename T>
void destruct_at(T* ptr) {
  ptr->T::~T();
}

}  // namespace psl
