#pragma once

#include <pine/psl/algorithm.h>
#include <pine/psl/memory.h>
#include <pine/psl/check.h>
#include <pine/psl/math.h>
#include <pine/psl/new.h>

#include <initializer_list>

namespace psl {

template <typename T>
struct default_allocator {
  T* alloc(size_t size) const {
    auto ptr = static_cast<T*>(::operator new(sizeof(T) * size));
    psl_check(ptr != nullptr);
    return ptr;
  }
  void free(T* ptr) const {
    psl_check(ptr != nullptr);
    ::operator delete(ptr);
  }

  template <typename... Args>
  void construct_at(T* ptr, Args&&... args) const {
    psl_check(ptr != nullptr);
    psl::construct_at(ptr, psl::forward<Args>(args)...);
  }

  void destruct_at(T* ptr) const {
    psl_check(ptr != nullptr);
    psl::destruct_at(ptr);
  }
};

template <typename T, typename Allocator>
class VectorBase {
public:
  using ValueType = T;

  using Iterator = T*;
  using ConstIterator = const T*;

  ~VectorBase() {
    clear();
  }

  VectorBase() = default;

  explicit VectorBase(size_t len) {
    resize(len);
  }
  VectorBase(size_t len, const T& x) : VectorBase{len} {
    fill(*this, x);
  }
  template <Range ARange>
  explicit VectorBase(ARange&& range) : VectorBase(psl::size(range)) {
    copy(begin(), range);
  }
  template <ForwardIterator It>
  VectorBase(It first, It last) : VectorBase{psl::make_range(first, last)} {
  }

  VectorBase(const VectorBase& rhs) : VectorBase{} {
    *this = rhs;
  }
  VectorBase(VectorBase&& rhs) : VectorBase{} {
    *this = move(rhs);
  }

  VectorBase& operator=(const VectorBase& rhs) {
    allocator = rhs.allocator;
    resize(rhs.size());
    copy(begin(), rhs);

    return *this;
  }
  VectorBase& operator=(VectorBase&& rhs) {
    psl::swap(ptr, rhs.ptr);
    psl::swap(len, rhs.len);
    psl::swap(reserved, rhs.reserved);
    psl::swap(allocator, rhs.allocator);
    return *this;
  }

  template <Range ARange>
  void assign_from(ARange&& input) {
    resize(psl::size(input));
    copy(begin(), input);
  }

  void push_back(T x) {
    reserve(size() + 1);
    len += 1;
    allocator.construct_at(&back(), psl::move(x));
  }
  void push_front(T x) {
    insert(begin(), psl::move(x));
  }

  template <typename... Args>
  void emplace_back(Args&&... args) {
    reserve(size() + 1);
    len += 1;
    allocator.construct_at(&back(), T{forward<Args>(args)...});
  }

  void pop_back() {
    resize(size() - 1);
  }
  void pop_front() {
    erase(begin());
  }

  Iterator insert(Iterator it, T x) {
    auto dist = it - begin();
    push_back(psl::move(x));
    it = begin() + dist;

    auto i = psl::prev(end());
    for (; i > it; --i)
      psl::swap(*psl::prev(i), *i);

    return it;
  }
  template <Range ARange>
  void insert(Iterator it, ARange&& range) {
    auto first = psl::begin(range);
    auto last = psl::end(range);
    while (first != last) {
      it = psl::next(insert(it, *first));
      ++first;
    }
  }

  void erase(Iterator it) {
    if (it == end())
      return;
    while (psl::next(it) != end()) {
      auto next = psl::next(it);
      psl::swap(*it, *next);
      ++it;
    }
    pop_back();
  }

  void resize(size_t nlen) {
    reserve(nlen);
    for (size_t i = size(); i < nlen; ++i)
      allocator.construct_at(&ptr[i]);
    for (size_t i = nlen; i < size(); ++i)
      allocator.destruct_at(&ptr[i]);
    len = nlen;
  }

  void reserve(size_t nreserved) {
    nreserved = roundup2(nreserved);
    if (nreserved <= reserved)
      return;

    auto nptr = allocator.alloc(nreserved);
    if (ptr) {
      psl::memcpy(nptr, ptr, size() * sizeof(T));
      allocator.free(ptr);
    }

    ptr = nptr;
    reserved = nreserved;
  }

  void clear() {
    for (size_t i = 0; i < size(); ++i)
      allocator.destruct_at(&ptr[i]);
    len = 0;
  }
  void reset() {
    clear();
    allocator.free(ptr);
    ptr = nullptr;
    reserved = 0;
  }

  T& operator[](size_t i) {
    return ptr[i];
  }
  const T& operator[](size_t i) const {
    return ptr[i];
  }

  Iterator begin() {
    return ptr;
  }
  Iterator end() {
    return ptr + size();
  }
  ConstIterator begin() const {
    return ptr;
  }
  ConstIterator end() const {
    return ptr + size();
  }
  T& front() {
    return ptr[0];
  }
  const T& front() const {
    return ptr[0];
  }
  T& back() {
    return ptr[size() - 1];
  }
  const T& back() const {
    return ptr[size() - 1];
  }

  size_t size() const {
    return len;
  }
  size_t capacity() const {
    return reserved;
  }

  const T* data() const {
    return ptr;
  }
  T* data() {
    return ptr;
  }

  template <typename U, typename UDeleter>
  requires EqualityComparable<T, U>
  bool operator==(const VectorBase<U, UDeleter>& rhs) const {
    if (size() != rhs.size())
      return false;
    for (size_t i = 0; i < size(); i++)
      if (ptr[i] != rhs[i])
        return false;
    return true;
  }
  template <typename U, typename UDeleter>
  requires EqualityComparable<T, U>
  bool operator!=(const VectorBase<U, UDeleter>& rhs) const {
    return !((*this) == rhs);
  }

protected:
  T* ptr = nullptr;
  size_t len = 0;
  size_t reserved = 0;

  Allocator allocator;
};

template <typename T>
using vector = VectorBase<T, default_allocator<T>>;

template <typename... Us, typename... Ts>
requires same_type<Ts...>
auto vector_of(Ts... xs) {
  if constexpr (sizeof...(Us) > 1) {
    static_assert(psl::deferred_bool<false, Ts...>, "Only one type-specifier is allowed");
  } else if constexpr (sizeof...(Us) == 1) {
    using T = FirstTypeT<Us...>;
    if constexpr (sizeof...(Ts) == 0)
      return vector<T>{};
    else
      return vector<T>{std::initializer_list<T>{psl::move(xs)...}};
  } else {
    if constexpr (sizeof...(Ts) == 0)
      static_assert(psl::deferred_bool<false, Ts...>,
                    "When the parameter pack can be null, please specify the vector type");
    else {
      using T = FirstTypeT<Ts...>;
      return vector<T>{std::initializer_list<T>{psl::move(xs)...}};
    }
  }
}

}  // namespace psl
