#pragma once

#include <pine/psl/type_traits.h>
#include <pine/psl/utility.h>
#include <pine/psl/stdint.h>
#include <pine/psl/new.h>

namespace psl {

inline constexpr void memcpy(void* dst, const void* src, size_t size) {
  auto csrc = static_cast<const char*>(src);
  auto cdst = static_cast<char*>(dst);
  for (size_t i = 0; i != size; i++)
    cdst[i] = csrc[i];
}

inline constexpr void memset(void* dst, char value, size_t size) {
  auto cdst = static_cast<char*>(dst);
  for (size_t i = 0; i != size; i++)
    cdst[i] = value;
}

void free(void* ptr);

template <typename T, typename... Args>
void construct_at(T* ptr, Args&&... args) {
  ::new (ptr) T(psl::forward<Args>(args)...);
}

template <typename T>
void destruct_at(T* ptr) {
  ptr->T::~T();
}

template <typename T>
struct DefaultDeleter {
  template <typename U>
  using ChangeBasis = DefaultDeleter<U>;

  DefaultDeleter() = default;
  template <typename U>
  requires IsConvertible<U*, T*>
  DefaultDeleter(const DefaultDeleter<U>&) {
  }

  void operator()(RemoveExtent<T>* ptr) const {
    if constexpr (IsArray<T>)
      delete[] ptr;
    else
      delete ptr;
  }
};

template <typename As, typename T>
requires(sizeof(As) == sizeof(T))
As bitcast(T&& x) {
  alignas(T) unsigned char storage[sizeof(T)];
  psl::memcpy(storage, &x, sizeof(T));
  return *reinterpret_cast<As*>(storage);
}

template <typename T, typename Deleter = DefaultDeleter<T>>
class unique_ptr {
public:
  using Pointer = RemoveExtent<T>*;
  using Reference = RemoveExtent<T>&;

  template <typename U, typename UDeleter>
  friend class unique_ptr;

  ~unique_ptr() {
    reset();
  }

  unique_ptr() = default;
  unique_ptr(nullptr_t) {
  }
  explicit unique_ptr(Pointer ptr, Deleter deleter = {}) : ptr{ptr}, deleter{deleter} {
  }

  unique_ptr(const unique_ptr&) = delete;
  unique_ptr& operator=(const unique_ptr&) = delete;

  unique_ptr(unique_ptr&& rhs) : unique_ptr() {
    *this = psl::move(rhs);
  }
  unique_ptr& operator=(unique_ptr&& rhs) {
    take(rhs);
    return *this;
  }

  template <typename U>
  requires(IsConvertible<U*, T*> && IsConvertible<ChangeBasisT<Deleter, U>, Deleter>)
  unique_ptr(unique_ptr<U, ChangeBasisT<Deleter, U>>&& rhs) : unique_ptr() {
    *this = psl::move(rhs);
  }
  template <typename U>
  requires(IsConvertible<U*, T*> && IsConvertible<ChangeBasisT<Deleter, U>, Deleter>)
  unique_ptr& operator=(unique_ptr<U, ChangeBasisT<Deleter, U>>&& rhs) {
    take(rhs);
    return *this;
  }

  Reference operator*() const {
    return *ptr;
  }
  Pointer operator->() const {
    return ptr;
  }
  Reference operator[](size_t i) const {
    static_assert(IsArray<T>, "operator[] can only as used when the underlying type is array");
    return ptr[i];
  }

  explicit operator bool() const {
    return ptr != nullptr;
  }

  Pointer get() const {
    return ptr;
  }

  Pointer release() {
    return psl::exchange(ptr, nullptr);
  }

  void reset(Pointer p = nullptr) {
    if (ptr != nullptr)
      deleter(ptr);
    ptr = p;
  }

  template <typename U, typename UDeleter>
  friend bool operator==(const unique_ptr& lhs, const unique_ptr<U, UDeleter>& rhs) {
    return lhs.get() == rhs.get();
  }
  template <typename U, typename UDeleter>
  friend bool operator!=(const unique_ptr& lhs, const unique_ptr<U, UDeleter>& rhs) {
    return lhs.get() != rhs.get();
  }
  template <typename U, typename UDeleter>
  friend bool operator>(const unique_ptr& lhs, const unique_ptr<U, UDeleter>& rhs) {
    return lhs.get() > rhs.get();
  }
  template <typename U, typename UDeleter>
  friend bool operator<(const unique_ptr& lhs, const unique_ptr<U, UDeleter>& rhs) {
    return lhs.get() < rhs.get();
  }
  bool operator==(nullptr_t) const {
    return ptr == nullptr;
  }
  bool operator!=(nullptr_t) const {
    return ptr != nullptr;
  }

private:
  template <typename U, typename UDeleter>
  void take(unique_ptr<U, UDeleter>& rhs) {
    this->~unique_ptr();
    ptr = psl::exchange(rhs.ptr, nullptr);
    deleter = psl::move(rhs.deleter);
  }

  Pointer ptr = nullptr;
  Deleter deleter = {};
};

template <typename T, typename... Args>
unique_ptr<T> make_unique(Args&&... args) {
  return unique_ptr<T>{new T{forward<Args>(args)...}};
}

template <typename T, typename Deleter = DefaultDeleter<T>>
class shared_ptr {
public:
  using Pointer = RemoveExtent<T>*;
  using Reference = RemoveExtent<T>&;

  template <typename U, typename UDeleter>
  friend class shared_ptr;

  ~shared_ptr() {
    decrement();
  }

  shared_ptr() = default;

  shared_ptr(nullptr_t) {
  }
  explicit shared_ptr(Pointer ptr, Deleter deleter = {})
      : ptr(ptr), deleter(deleter), refcount(new size_t{1}) {
  }

  shared_ptr(const shared_ptr& rhs) : shared_ptr() {
    *this = rhs;
  }
  shared_ptr(shared_ptr&& rhs) : shared_ptr() {
    *this = psl::move(rhs);
  }
  shared_ptr& operator=(const shared_ptr& rhs) {
    copy(rhs);
    return *this;
  }
  shared_ptr& operator=(shared_ptr&& rhs) {
    take(rhs);
    return *this;
  }

  template <typename U>
  requires(IsConvertible<U*, T*> && IsConvertible<ChangeBasisT<Deleter, U>, Deleter>)
  shared_ptr(const shared_ptr<U, ChangeBasisT<Deleter, U>>& rhs) : shared_ptr() {
    *this = rhs;
  }
  template <typename U>
  requires(IsConvertible<U*, T*> && IsConvertible<ChangeBasisT<Deleter, U>, Deleter>)
  shared_ptr(shared_ptr<U, ChangeBasisT<Deleter, U>>&& rhs) : shared_ptr() {
    *this = psl::move(rhs);
  }
  template <typename U>
  requires(IsConvertible<U*, T*> && IsConvertible<ChangeBasisT<Deleter, U>, Deleter>)
  shared_ptr& operator=(const shared_ptr<U, ChangeBasisT<Deleter, U>>& rhs) {
    copy(rhs);
    return *this;
  }
  template <typename U>
  requires(IsConvertible<U*, T*> && IsConvertible<ChangeBasisT<Deleter, U>, Deleter>)
  shared_ptr& operator=(shared_ptr<U, ChangeBasisT<Deleter, U>>&& rhs) {
    take(rhs);
    return *this;
  }

  Reference operator*() const {
    return *ptr;
  }
  Pointer operator->() const {
    return ptr;
  }
  Reference operator[](size_t i) const {
    static_assert(IsArray<T>, "operator[] can only as used when the underlying type is array");
    return ptr[i];
  }

  explicit operator bool() const {
    return get() != Pointer();
  }

  Pointer get() const {
    return ptr;
  }

  void reset(Pointer p = {}) {
    decrement();

    ptr = p;
    refcount = new size_t(1);
  }

  template <typename U, typename UDeleter>
  friend bool operator==(const shared_ptr& lhs, const shared_ptr<U, UDeleter>& rhs) {
    return lhs.get() == rhs.get();
  }
  template <typename U, typename UDeleter>
  friend bool operator!=(const shared_ptr& lhs, const shared_ptr<U, UDeleter>& rhs) {
    return lhs.get() != rhs.get();
  }
  template <typename U, typename UDeleter>
  friend bool operator>(const shared_ptr& lhs, const shared_ptr<U, UDeleter>& rhs) {
    return lhs.get() > rhs.get();
  }
  template <typename U, typename UDeleter>
  friend bool operator<(const shared_ptr& lhs, const shared_ptr<U, UDeleter>& rhs) {
    return lhs.get() < rhs.get();
  }
  bool operator==(nullptr_t) const {
    return ptr == nullptr;
  }
  bool operator!=(nullptr_t) const {
    return ptr != nullptr;
  }

private:
  template <typename U, typename UDeleter>
  void copy(const shared_ptr<U, UDeleter>& rhs) {
    decrement();

    ptr = rhs.ptr;
    deleter = rhs.deleter;
    refcount = rhs.refcount;

    if (refcount)
      ++(*refcount);
  }

  template <typename U, typename UDeleter>
  void take(shared_ptr<U, UDeleter>& rhs) {
    decrement();

    ptr = psl::exchange(rhs.ptr, nullptr);
    deleter = psl::move(rhs.deleter);
    refcount = psl::exchange(rhs.refcount, nullptr);
  }

  void decrement() {
    if (ptr != Pointer()) {
      if (--(*refcount) == 0) {
        deleter(ptr);
        delete refcount;
      }
    }
  }

  Pointer ptr = {};
  Deleter deleter = {};
  size_t* refcount = nullptr;
};
template <typename T, typename... Args>
shared_ptr<T> make_shared(Args&&... args) {
  return shared_ptr<T>{new T{psl::forward<Args>(args)...}};
}

template <typename T>
struct ref {
  using BaseType = T;
  ref(T& x) : ptr{&x} {
  }

  template <typename U>
  ref& operator=(U rhs) {
    *ptr = psl::move(rhs);
    return *this;
  }

  operator T&() {
    return *ptr;
  }
  operator const T&() const {
    return *ptr;
  }

private:
  T* ptr;
};
template <typename T>
ref(T& x) -> ref<T>;

template <typename T>
struct _is_psl_ref : FalseType {};
template <typename T>
struct _is_psl_ref<ref<T>> : TrueType {};
template <typename T>
constexpr bool is_psl_ref = _is_psl_ref<T>::value;

template <typename T>
struct Box {
  Box() = default;
  Box(T x) : ptr(psl::make_unique<T>(psl::move(x))) {
  }
  Box(Box&&) = default;
  Box(const Box& b) {
    if (b.ptr)
      ptr = psl::make_unique<T>(*b);
  }
  Box& operator=(Box b) {
    ptr = move(b.ptr);
    return *this;
  }

  T& operator*() {
    return *ptr;
  }
  const T& operator*() const {
    return *ptr;
  }
  T* operator->() {
    return ptr.get();
  }
  const T* operator->() const {
    return ptr.get();
  }
  explicit operator bool() const {
    return static_cast<bool>(ptr);
  }
  void reset() {
    ptr.reset();
  }

private:
  psl::unique_ptr<T> ptr;
};

}  // namespace psl
