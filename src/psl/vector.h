#pragma once
#include <psl/algorithm.h>
#include <psl/memory.h>
#include <psl/check.h>
#include <psl/math.h>
#include <psl/new.h>

#include <initializer_list>

namespace psl {

template <typename T>
struct default_allocator {
  T* alloc(size_t count) const {
    auto ptr = (T*)::operator new(sizeof(T) * count);
    psl_check(ptr != nullptr);
    return ptr;
  }
  void free(T* ptr) const { ::operator delete(ptr); }

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

template <typename T>
struct context_allocator {
  T* alloc(size_t count) const {
    auto ptr = (T*)context_alloc(sizeof(T) * count);
    psl_check(ptr != nullptr);
    return ptr;
  }
  void free(T* ptr) const { context_free(ptr); }

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

template <typename T, size_t capacity>
struct static_allocator {
  T* alloc(size_t size [[maybe_unused]]) const {
    psl_check(size <= capacity);
    return reinterpret_cast<T*>(storage.ptr());
  }
  void free(T* ptr [[maybe_unused]]) const {}

  template <typename... Args>
  void construct_at(T* ptr, Args&&... args) const {
    psl_check(ptr != nullptr);
    psl::construct_at(ptr, psl::forward<Args>(args)...);
  }

  void destruct_at(T* ptr) const {
    psl_check(ptr != nullptr);
    psl::destruct_at(ptr);
  }

  psl::Storage<sizeof(T) * capacity, alignof(T)> storage;
};

// TODO: use relocate

template <typename T, typename Allocator = default_allocator<T>>
class vector {
 public:
  using ValueType = T;

  using Iterator = T*;
  using ConstIterator = const T*;

  ~vector() { reset(); }

  vector() = default;

  explicit vector(size_t len) { resize(len); }
  struct _reserve_size {};
  inline static constexpr _reserve_size reserve_size{};
  vector(size_t len, _reserve_size) { reserve(len); }

  template <Range ARange>
  explicit vector(ARange&& range) {
    if constexpr (psl::has_size<ARange>) {
      reserve(psl::size(range));
      psl::copy_inplace(begin(), range);
      len = psl::size(range);
    } else {
      psl::insert([&](const auto& x) { push_back(x); }, range);
    }
  }
  template <ForwardIterator It>
  vector(It first, It last) : vector(psl::range(first, last)) {}

  vector(const vector& rhs) : vector{} { *this = rhs; }
  vector(vector&& rhs) : vector{} { *this = move(rhs); }

  vector& operator=(const vector& rhs) {
    clear();
    allocator = rhs.allocator;
    reserve(rhs.size());
    psl::copy_inplace(begin(), rhs);
    len = rhs.size();

    return *this;
  }
  vector& operator=(vector&& rhs) {
    psl::swap(ptr, rhs.ptr);
    psl::swap(len, rhs.len);
    psl::swap(reserved, rhs.reserved);
    psl::swap(allocator, rhs.allocator);
    return *this;
  }

  void push_back(T x) {
    reserve(size() + 1);
    len += 1;
    allocator.construct_at(&back(), psl::move(x));
  }
  template <typename... Ts>
  requires(sizeof...(Ts) > 1)
  void push_back(Ts... xs) {
    (push_back(MOVE(xs)), ...);
  }
  void push_front(T x) { insert(begin(), psl::move(x)); }

  template <typename... Args>
  void emplace_back(Args&&... args) {
    reserve(size() + 1);
    len += 1;
    allocator.construct_at(&back(), T(forward<Args>(args)...));
  }

  void pop_back() { resize_less(size() - 1); }
  T consume_back() {
    auto x = back();
    pop_back();
    return x;
  }
  void pop_back(size_t n) {
    psl_check(n <= size());
    resize_less(size() - n);
  }
  void pop_front() { erase(begin()); }
  T consume_front() {
    auto x = front();
    pop_front();
    return x;
  }

  Iterator insert(Iterator it, T x) {
    auto dist = it - begin();
    push_back(psl::move(x));
    it = begin() + dist;

    auto i = psl::prev(end());
    for (; i > it; --i) psl::swap(*psl::prev(i), *i);

    return it;
  }
  template <Range ARange>
  void insert_range(Iterator it, ARange&& range) {
    auto p = it - begin();
    auto suffix = vector(it, end());
    auto nlen = size() + psl::size(range);
    resize_less(p);
    reserve(nlen);
    psl::copy_inplace(begin() + p, range);
    psl::copy_inplace(begin() + p + psl::size(range), suffix);
    len = nlen;
  }

  auto erase(Iterator it) {
    if (it == end()) return it;
    auto offset = it - begin();
    while (psl::next(it) != end()) {
      auto next = psl::next(it);
      psl::swap(*it, *next);
      ++it;
    }
    pop_back();
    return begin() + offset;
  }
  auto erase_range(Iterator first, Iterator last) {
    psl_check(first <= last);
    if (first == last) return first;
    auto offset = first - begin();
    while (last != end()) {
      psl::swap(*first, *last);
      ++first;
      ++last;
    }

    resize(size() - (last - first));
    return begin() + offset;
  }

  void resize(size_t nlen) {
    reserve(nlen);
    for (size_t i = size(); i < nlen; ++i) allocator.construct_at(&ptr[i]);
    for (size_t i = nlen; i < size(); ++i) allocator.destruct_at(&ptr[i]);
    len = nlen;
  }
  void resize_less(size_t nlen) {
    psl_check(nlen <= len);
    for (size_t i = nlen; i < size(); ++i) allocator.destruct_at(&ptr[i]);
    len = nlen;
  }

  void reserve(size_t nreserved) {
    nreserved = roundup2(nreserved);
    if (nreserved <= reserved) return;

    auto nptr = allocator.alloc(nreserved);
    if (ptr) {
      psl::memcpy(nptr, ptr, size() * sizeof(T));
      allocator.free(ptr);
    }

    ptr = nptr;
    reserved = nreserved;
  }

  void clear() {
    for (size_t i = 0; i < size(); ++i) allocator.destruct_at(&ptr[i]);
    len = 0;
  }
  void reset() {
    clear();
    allocator.free(ptr);
    ptr = nullptr;
    reserved = 0;
  }

  T& operator[](size_t i) { return ptr[i]; }
  const T& operator[](size_t i) const { return ptr[i]; }

  Iterator begin() { return ptr; }
  Iterator end() { return ptr + size(); }
  ConstIterator begin() const { return ptr; }
  ConstIterator end() const { return ptr + size(); }
  T& front() { return ptr[0]; }
  const T& front() const { return ptr[0]; }
  T& back() { return ptr[size() - 1]; }
  const T& back() const { return ptr[size() - 1]; }

  size_t size() const { return len; }
  size_t byte_size() const { return size() * sizeof(T); }
  size_t capacity() const { return reserved; }

  const T* data() const { return ptr; }
  T* data() { return ptr; }

  template <typename U, typename UDeleter>
  requires EqualityComparable<T, U>
  bool operator==(const vector<U, UDeleter>& rhs) const {
    if (size() != rhs.size()) return false;
    for (size_t i = 0; i < size(); i++)
      if (ptr[i] != rhs[i]) return false;
    return true;
  }
  template <typename U, typename UDeleter>
  requires EqualityComparable<T, U>
  bool operator!=(const vector<U, UDeleter>& rhs) const {
    return !((*this) == rhs);
  }

  void _set_size(size_t nlen) { len = nlen; }

 protected:
  T* ptr = nullptr;
  size_t len = 0;
  size_t reserved = 0;

  Allocator allocator;
};

template <typename T, size_t capacity>
struct static_vector : vector<T, static_allocator<T, capacity>> {
  using vector<T, static_allocator<T, capacity>>::vector;
};

template <typename T>
using context_vector = vector<T, context_allocator<T>>;

template <typename T, size_t _max_static_capacity>
struct smart_vector {
 public:
  using ValueType = T;

  using Iterator = T*;
  using ConstIterator = const T*;

  static constexpr size_t max_static_capacity = psl::roundup2(_max_static_capacity);

  smart_vector() = default;

  explicit smart_vector(size_t len) {
    if (len <= max_static_capacity) {
      st = St(len);
    } else {
      dy = Dy(len);
      overflowed = true;
    }
  }
  template <Range ARange>
  explicit smart_vector(ARange&& range) {
    if (psl::size(range) <= max_static_capacity) {
      st = St(FWD(range));
    } else {
      dy = Dy(FWD(range));
      overflowed = true;
    }
  }
  template <ForwardIterator It>
  smart_vector(It first, It last) {
    if (psl::distance(first, last) <= max_static_capacity) {
      st = St(first, last);
    } else {
      dy = Dy(first, last);
      overflowed = true;
    }
  }

  void push_back(T x) {
    if (overflowed) {
      dy.push_back(MOVE(x));
    } else {
      if (st.size() + 1 <= max_static_capacity) {
        st.push_back(MOVE(x));
      } else {
        switch_to_dy(st.size() + 1);
        dy._set_size(dy.size() + 1);
        psl::construct_at(&dy.back(), psl::move(x));
      }
    }
  }

  void pop_back() { resize_less(size() - 1); }

  void resize(size_t nlen) {
    if (overflowed) {
      dy.resize(nlen);
    } else {
      if (nlen > max_static_capacity) {
        switch_to_dy(nlen, true);
      } else {
        st.resize(nlen);
      }
    }
  }
  void resize_less(size_t nlen) {
    psl_check(nlen <= size());
    if (overflowed)
      dy.resize_less(nlen);
    else
      st.resize_less(nlen);
  }

  void reserve(size_t nreserved) {
    if (overflowed) {
      dy.reserve(nreserved);
    } else {
      if (nreserved > max_static_capacity) {
        switch_to_dy(nreserved);
      }
    }
  }

  void clear() {
    dy.clear();
    st.clear();
  }
  void reset() {
    dy.reset();
    st.reset();
  }

  T& operator[](size_t i) { return overflowed ? dy[i] : st[i]; }
  const T& operator[](size_t i) const { return overflowed ? dy[i] : st[i]; }

  Iterator begin() { return overflowed ? dy.begin() : st.begin(); }
  Iterator end() { return overflowed ? dy.end() : st.end(); }
  ConstIterator begin() const { return overflowed ? dy.begin() : st.begin(); }
  ConstIterator end() const { return overflowed ? dy.end() : st.end(); }
  T& front() { return overflowed ? dy.front() : st.front(); }
  const T& front() const { return overflowed ? dy.front() : st.front(); }
  T& back() { return overflowed ? dy.back() : st.back(); }
  const T& back() const { return overflowed ? dy.back() : st.back(); }

  size_t size() const { return overflowed ? dy.size() : st.size(); }
  size_t byte_size() const { return overflowed ? dy.byte_size() : st.byte_size(); }
  size_t capacity() const { return overflowed ? dy.capacity() : max_static_capacity; }

  const T* data() const { return overflowed ? dy.data() : st.data(); }
  T* data() { return overflowed ? dy.data() : st.data(); }

 private:
  void switch_to_dy(size_t new_size, bool zero_init = false) {
    psl_check(!overflowed);
    psl_check(new_size >= st.size());
    dy.reserve(new_size);
    psl::move_inplace(dy.begin(), st);
    if (zero_init) {
      for (size_t i = st.size(); i < new_size; i++) psl::construct_at(&dy[i]);
      dy._set_size(new_size);
    } else {
      dy._set_size(st.size());
    }
    st._set_size(0);
    overflowed = true;
  }

  using Dy = vector<T>;
  using St = static_vector<T, max_static_capacity>;
  Dy dy;
  St st;
  bool overflowed = false;
};

template <typename T>
vector<T> vector_n_of(size_t n, const T& x) {
  auto v = vector<T>();
  v.reserve(n);
  v._set_size(n);
  psl::fill_inplace(v, x);
  return v;
}

template <typename... Us, typename... Ts>
requires same_type<Ts...>
auto vector_of(Ts... xs) {
  if constexpr (sizeof...(Us) > 1) {
    static_assert(psl::deferred_bool<false, Ts...>, "Only one type-specifier is allowed");
  } else if constexpr (sizeof...(Us) == 1) {
    using T = FirstType<Us...>;
    if constexpr (sizeof...(Ts) == 0)
      return vector<T>{};
    else {
      auto r = vector<T>();
      r.reserve(sizeof...(xs));
      (r.push_back(MOVE(xs)), ...);
      return r;
    }
  } else {
    if constexpr (sizeof...(Ts) == 0)
      static_assert(psl::deferred_bool<false, Ts...>,
                    "When the parameter pack can be null, please specify the vector type");
    else {
      using T = FirstType<Ts...>;
      auto r = vector<T>();
      r.reserve(sizeof...(xs));
      (r.push_back(MOVE(xs)), ...);
      return r;
    }
  }
}

auto transform_vector(Range auto&& xs, auto f) {
  using R = RemoveReference<decltype(f(*xs.begin()))>;
  auto res = vector<R>(xs.size(), vector<R>::reserve_size);
  for (const auto& x : xs) res.push_back(f(x));
  return res;
}

}  // namespace psl
