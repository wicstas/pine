#pragma once

#include <psl/type_traits.h>
#include <psl/utility.h>
#include <psl/stdint.h>
#include <psl/new.h>

namespace psl {

void* context_alloc(size_t size);
void context_free(const void* ptr);

template <typename T, typename... Args>
T* context_new(Args&&... args) {
  auto ptr = (T*)context_alloc(sizeof(T));
  new (ptr) T(FWD(args)...);
  return ptr;
}
template <typename T>
void context_delete(T* ptr) {
  if (ptr) {
    ptr->~T();
    context_free(ptr);
  }
}

void push_context_allocator(struct pm_allocator allocator);
void pop_context_allocator();

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

template <typename T>
struct default_deleter {
  default_deleter() = default;
  template <DerivedFrom<T> U>
  default_deleter(const default_deleter<U>&) {
  }

  void operator()(T* ptr) const {
    delete ptr;
  }
};
template <typename T>
struct context_deleter {
  context_deleter() = default;
  template <DerivedFrom<T> U>
  context_deleter(const context_deleter<U>&) {
  }

  void operator()(T* ptr) const {
    context_delete(ptr);
  }
};
template <typename T>
struct no_free_deleter {
  no_free_deleter() = default;
  template <DerivedFrom<T> U>
  no_free_deleter(const no_free_deleter<U>&) {
  }

  void operator()(T* ptr) const {
    ptr->~T();
  }
};

template <typename As, typename T>
requires(sizeof(As) == sizeof(T))
As bitcast(T&& x) {
  alignas(T) unsigned char storage[sizeof(T)];
  psl::memcpy(storage, &x, sizeof(T));
  return *reinterpret_cast<As*>(storage);
}
template <typename As, typename T>
requires(sizeof(As) == sizeof(T))
As bitcast_unsafe(T&& x) {
  return *(As*)&x;
}

template <typename T, typename Deleter = default_deleter<T>>
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
    take(rhs);
  }
  unique_ptr& operator=(unique_ptr&& rhs) {
    take(rhs);
    return *this;
  }

  template <DerivedFrom<T> U, typename DeleterU>
  requires(convertible<DeleterU, Deleter>)
  unique_ptr(unique_ptr<U, DeleterU>&& rhs) : unique_ptr() {
    take(rhs);
  }
  template <DerivedFrom<T> U, typename DeleterU>
  requires(convertible<Deleter, DeleterU>)
  unique_ptr& operator=(unique_ptr<U, DeleterU>&& rhs) {
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
    static_assert(psl::is_array<T>,
                  "operator[] can only as used when the underlying type is array");
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

protected:
  template <typename U, typename UDeleter>
  void take(unique_ptr<U, UDeleter>& rhs) {
    this->~unique_ptr();
    ptr = psl::exchange(rhs.ptr, nullptr);
    deleter = psl::move(rhs.deleter);
  }

  Pointer ptr = nullptr;

public:
  Deleter deleter = {};
};
template <typename T>
using context_unique_ptr = unique_ptr<T, context_deleter<T>>;
template <typename T, typename... Args>
context_unique_ptr<T> make_context_unique(Args&&... args) {
  return context_unique_ptr<T>(context_new<T>(FWD(args)...));
}
template <typename T, typename... Args>
unique_ptr<T> make_unique(Args&&... args) {
  return unique_ptr<T>(new T(FWD(args)...));
}

template <typename T, typename Deleter = default_deleter<T>>
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
      : deleter(deleter), ptr(ptr), refcount(new size_t(1)) {
  }

  shared_ptr(const shared_ptr& rhs) : shared_ptr() {
    copy(rhs);
  }
  shared_ptr(shared_ptr&& rhs) : shared_ptr() {
    take(rhs);
  }
  shared_ptr& operator=(shared_ptr rhs) {
    take(rhs);
    return *this;
  }

  template <DerivedFrom<T> U>
  shared_ptr(shared_ptr<U> rhs) : shared_ptr() {
    take(rhs);
  }
  template <DerivedFrom<T> U>
  shared_ptr& operator=(shared_ptr<U> rhs) {
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
    static_assert(psl::is_array<T>,
                  "operator[] can only as used when the underlying type is array");
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

  template <typename U>
  friend bool operator==(const shared_ptr& lhs, const shared_ptr<U>& rhs) {
    return lhs.get() == rhs.get();
  }
  template <typename U>
  friend bool operator!=(const shared_ptr& lhs, const shared_ptr<U>& rhs) {
    return lhs.get() != rhs.get();
  }
  template <typename U>
  friend bool operator>(const shared_ptr& lhs, const shared_ptr<U>& rhs) {
    return lhs.get() > rhs.get();
  }
  template <typename U>
  friend bool operator<(const shared_ptr& lhs, const shared_ptr<U>& rhs) {
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

    deleter = rhs.deleter;
    ptr = rhs.ptr;
    refcount = rhs.refcount;

    if (refcount)
      ++(*refcount);
  }

  template <typename U, typename UDeleter>
  void take(shared_ptr<U, UDeleter>& rhs) {
    decrement();

    deleter = psl::move(rhs.deleter);
    ptr = psl::exchange(rhs.ptr, nullptr);
    refcount = psl::exchange(rhs.refcount, nullptr);
  }

  void decrement() {
    if (ptr != Pointer()) {
      if (--(*refcount) == 0) {
        deleter(ptr);
        ptr = nullptr;
        delete refcount;
      }
    }
  }

  Deleter deleter;
  Pointer ptr = nullptr;
  size_t* refcount = nullptr;
};
template <typename T>
using context_shared_ptr = shared_ptr<T, context_deleter<T>>;
template <typename T, typename... Args>
context_shared_ptr<T> make_context_shared(Args&&... args) {
  return context_shared_ptr<T>(context_new<T>(FWD(args)...));
}
template <typename T, typename... Args>
shared_ptr<T> make_shared(Args&&... args) {
  return shared_ptr<T>(new T(FWD(args)...));
}

struct pm_allocator {
  struct pm_concept {
    virtual ~pm_concept() = default;
    virtual void* alloc(size_t size) = 0;
    virtual void free(void* ptr) = 0;
  };
  template <typename T>
  struct pm_model : pm_concept {
    pm_model(T base) : base(psl::move(base)) {
    }
    void* alloc(size_t size) override {
      return base.alloc(size);
    }
    void free(void* ptr) override {
      return base.free(ptr);
    }

  private:
    T base;
  };

  template <typename T>
  pm_allocator(T x) : model(psl::make_unique<pm_model<T>>(MOVE(x))) {
  }

  void* alloc(size_t size) {
    return model->alloc(size);
  }
  void free(void* ptr) {
    model->free(ptr);
  }

private:
  unique_ptr<pm_concept> model;
};

struct global_allocator {
  void* alloc(size_t size) {
    return ::operator new(size);
  }
  void free(void* ptr) {
    ::operator delete(ptr);
  }
};

template <typename T>
struct Ref {
  using BaseType = T;
  Ref(T& x) : ptr{&x} {
  }

  template <typename U>
  Ref& operator=(U rhs) {
    *ptr = psl::move(rhs);
    return *this;
  }

  T& operator*() {
    return *ptr;
  }
  const T& operator*() const {
    return *ptr;
  }

private:
  T* ptr;
};

template <typename T>
auto ref(T& x) {
  return Ref<T>(x);
}

template <typename T>
struct _is_psl_ref : FalseType {};
template <typename T>
struct _is_psl_ref<Ref<T>> : TrueType {};
template <typename T>
constexpr bool is_psl_ref = _is_psl_ref<T>::value;

template <typename T>
struct Box {
  static Box default_value() {
    return Box(T());
  }
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

template <size_t bytes, size_t align_>
struct Storage {
  template <typename T>
  void operator=(const T& x) {
    static_assert(!psl::same_as<T, Storage>, "");
    static_assert(align_ % alignof(T) == 0, "");
    psl::memcpy(data, &x, sizeof(T));
  }

  template <typename T>
  T& as() {
    static_assert(align_ % alignof(T) == 0, "");
    return *reinterpret_cast<T*>(data);
  }

  void* ptr() const {
    return const_cast<unsigned char*>(data);
  }

private:
  alignas(align_) unsigned char data[bytes];
};
template <size_t align_>
struct Storage<0, align_> {
  template <typename T>
  void operator=(const T&) {
  }

  template <typename T>
  T& as() {
    return *(T*)nullptr;
  }

  void* ptr() const {
    return nullptr;
  }
};

}  // namespace psl
