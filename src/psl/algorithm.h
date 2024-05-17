#pragma once

#include <psl/type_traits.h>
#include <psl/utility.h>
#include <psl/new.h>

namespace psl {

auto composite(auto f, auto g) {
  return [f = psl::move(f), g = psl::move(g)](auto&& x) { return f(g(FWD(x))); };
}

constexpr auto add_ = [](auto&& a, auto&& b) -> decltype(auto) { return a + b; };
constexpr auto sub_ = [](auto&& a, auto&& b) -> decltype(auto) { return a - b; };
constexpr auto mul_ = [](auto&& a, auto&& b) -> decltype(auto) { return a * b; };
constexpr auto div_ = [](auto&& a, auto&& b) -> decltype(auto) { return a / b; };
constexpr auto mod_ = [](auto&& a, auto&& b) -> decltype(auto) { return a % b; };
constexpr auto adde_ = [](auto&& a, auto&& b) -> decltype(auto) { return a += b; };
constexpr auto sube_ = [](auto&& a, auto&& b) -> decltype(auto) { return a -= b; };
constexpr auto mule_ = [](auto&& a, auto&& b) -> decltype(auto) { return a *= b; };
constexpr auto dive_ = [](auto&& a, auto&& b) -> decltype(auto) { return a /= b; };
constexpr auto mode_ = [](auto&& a, auto&& b) -> decltype(auto) { return a %= b; };
constexpr auto eq_ = [](auto&& a, auto&& b) -> decltype(auto) { return a == b; };
constexpr auto ne_ = [](auto&& a, auto&& b) -> decltype(auto) { return a != b; };
constexpr auto lt_ = [](auto&& a, auto&& b) -> decltype(auto) { return a < b; };
constexpr auto gt_ = [](auto&& a, auto&& b) -> decltype(auto) { return a > b; };
constexpr auto le_ = [](auto&& a, auto&& b) -> decltype(auto) { return a <= b; };
constexpr auto ge_ = [](auto&& a, auto&& b) -> decltype(auto) { return a >= b; };

template <typename T>
requires requires(T x) { x.begin(); }
auto begin(T&& x) {
  return x.begin();
}
template <typename T>
requires requires(T x) { x.end(); }
auto end(T&& x) {
  return x.end();
}
template <typename T, size_t N>
auto begin(T (&x)[N]) {
  return x;
}
template <typename T, size_t N>
auto end(T (&x)[N]) {
  return x + N;
}

template <typename T>
requires requires(T&& x) { x.size(); }
auto size(T&& x) {
  return x.size();
}
template <typename T, size_t N>
auto size(T[N]) {
  return N;
}

template <typename T>
concept Range = requires(T& x) {
  psl::begin(x);
  psl::end(x);
};
auto iters(Range auto&& range) {
  return psl::make_pair(psl::begin(range), psl::end(range));
}

template <typename T>
constexpr bool has_size = requires(T& x) { psl::size(x); };

template <Range T>
struct IteratorType {
  using Type = decltype(psl::begin(psl::declval<T>()));
};
template <Range T>
using IteratorTypeT = typename IteratorType<T>::Type;

template <RandomAccessIterator It>
auto distance(It first, It last, PriorityTag<1>) {
  return last - first;
}
template <ForwardIterator It>
auto distance(It first, It last, PriorityTag<0>) {
  auto dist = ptrdiff_t{0};
  for (; first != last; ++first)
    ++dist;
  return dist;
}
template <typename It>
auto distance(It first, It last) {
  return psl::distance(first, last, PriorityTag<1>{});
}

template <RandomAccessIterator It>
auto range(It first, It last) {
  struct Wrapper {
    It begin() const {
      return first;
    }
    It end() const {
      return last;
    }
    size_t size() const {
      return psl::distance(first, last);
    }

    It first, last;
  };

  return Wrapper{first, last};
}
template <typename It>
auto range(It first, It last) {
  struct Wrapper {
    It begin() const {
      return first;
    }
    It end() const {
      return last;
    }

    It first, last;
  };

  return Wrapper{first, last};
}
template <typename It>
auto range(It first, size_t length) {
  return range(first, first + length);
}

template <ForwardIterator It>
It next(It it) {
  return ++it;
}

template <ForwardIterator It>
It next(It it, size_t n) {
  if constexpr (psl::RandomAccessIterator<It>) {
    return it + n;
  } else {
    for (; n; --n)
      ++it;
    return it;
  }
}

template <BackwardIterator It>
It prev(It it) {
  return --it;
}

template <BackwardIterator It>
It prev(It it, size_t n) {
  if constexpr (psl::RandomAccessIterator<It>) {
    return it - n;
  } else {
    for (; n; --n)
      --it;
    return it;
  }
}

void copy(OutputIterator auto d_first, Range auto&& input) {
  auto first = psl::begin(input);
  auto last = psl::end(input);
  for (; first != last; ++first, ++d_first)
    *d_first = *first;
}

void move(OutputIterator auto d_first, Range auto&& input) {
  auto first = psl::begin(input);
  auto last = psl::end(input);
  for (; first != last; ++first, ++d_first)
    *d_first = psl::move(*first);
}

void copy_inplace(OutputIterator auto d_first, Range auto&& input) {
  auto first = psl::begin(input);
  auto last = psl::end(input);
  for (; first != last; ++first, ++d_first)
    psl::construct_at(&(*d_first), *first);
}

void move_inplace(OutputIterator auto d_first, Range auto&& input) {
  auto first = psl::begin(input);
  auto last = psl::end(input);
  for (; first != last; ++first, ++d_first)
    psl::construct_at(&(*d_first), psl::move(*first));
}

void fill(Range auto&& output, const auto& value) {
  auto first = psl::begin(output);
  auto last = psl::end(output);
  for (; first != last; ++first)
    *first = value;
}
void fill_f(Range auto&& output, const auto& generator) {
  auto first = psl::begin(output);
  auto last = psl::end(output);
  for (; first != last; ++first)
    *first = generator();
}

void fill_inplace(Range auto&& output, const auto& value) {
  auto first = psl::begin(output);
  auto last = psl::end(output);
  for (; first != last; ++first)
    psl::construct_at(&*first, value);
}

void insert(auto&& f, Range auto&& input) {
  auto first = psl::begin(input);
  auto last = psl::end(input);
  for (; first != last; ++first)
    f(*first);
}

namespace range_equal {
bool operator==(Range auto&& a, Range auto&& b) {
  if (psl::size(a) != psl::size(b))
    return false;
  auto a_it = psl::begin(a);
  auto b_it = psl::begin(b);
  while (a_it != psl::end(a)) {
    if (*a_it != *b_it)
      return false;
    ++a_it;
    ++b_it;
  }

  return true;
}
}  // namespace range_equal

struct StrictOrdering {};

template <typename T = void>
struct less : StrictOrdering {
  bool operator()(const T& l, const T& r) const {
    return l < r;
  }
};
template <>
struct less<void> : StrictOrdering {
  using is_transparent = void;
  bool operator()(const auto& l, const auto& r) const {
    return l < r;
  }
};

template <typename T = void>
struct greater : StrictOrdering {
  bool operator()(const T& l, const T& r) const {
    return l > r;
  }
};
template <>
struct greater<void> : StrictOrdering {
  using is_transparent = void;
  bool operator()(const auto& l, const auto& r) const {
    return l > r;
  }
};

template <typename T>
struct _less_than {
  bool operator()(const auto& x) const {
    return x < value;
  }
  T value;
};
template <typename T>
struct _equal_to {
  bool operator()(const auto& x) const {
    return x == value;
  }
  T value;
};
template <typename T>
struct _not_equal_to {
  bool operator()(const auto& x) const {
    return x != value;
  }
  T value;
};
template <typename T>
struct _less_or_equal_to {
  bool operator()(const auto& x) const {
    return x <= value;
  }
  T value;
};
template <typename T>
struct _greater_than {
  bool operator()(const auto& x) const {
    return x > value;
  }
  T value;
};
template <typename T>
struct _greater_or_equal_to {
  bool operator()(const auto& x) const {
    return x >= value;
  }
  T value;
};
template <typename T>
auto less_than(T value) {
  return _less_than<T>{psl::move(value)};
}
template <typename T>
auto greater_than(T value) {
  return _greater_than<T>{psl::move(value)};
}
template <typename T>
auto equal_to(T value) {
  return _equal_to<T>{psl::move(value)};
}
template <typename... Ts>
auto equal_to_any(Ts... values) {
  return [=](const auto& rhs) { return ((rhs == values) || ...); };
}
template <typename T>
auto less_or_equal_to(T value) {
  return _less_or_equal_to<T>{psl::move(value)};
}
template <typename T>
auto greater_or_equal_to(T value) {
  return _greater_or_equal_to<T>{psl::move(value)};
}

// Return the iterator to the first element not satisfying `pred`
auto lower_bound(Range auto&& range, auto&& pred) {
  auto first = psl::begin(range);
  auto last = psl::end(range);
  // If there is an element such that `pred` is not satisfied, it's always between first and
  // last

  while (first != last) {
    auto mid = first + psl::distance(first, last) / 2;

    if (pred(*mid))
      first = psl::next(mid);
    else
      last = mid;
  }

  return first;
}

auto find_if(Range auto&& range, auto&& f) {
  auto first = psl::begin(range);
  auto last = psl::end(range);
  for (; first != last; ++first)
    if (f(*first))
      return first;
  return last;
}

auto find(Range auto&& range, const auto& value) {
  return find_if(range, equal_to(value));
}

bool contains(Range auto&& range, const auto& value) {
  return psl::find(range, value) != psl::end(range);
}

void replace(Range auto&& range, const auto& old, const auto& new_) {
  auto first = psl::begin(range);
  auto last = psl::end(range);
  for (; first != last; ++first)
    if (*first == old)
      *first = new_;
}

size_t count_if(Range auto&& range, auto&& pred) {
  auto c = size_t(0);
  auto first = psl::begin(range);
  auto last = psl::end(range);
  for (; first != last; ++first)
    if (pred(*first))
      ++c;
  return c;
}
size_t count(Range auto&& range, const auto& x) {
  return count_if(range, equal_to(x));
}

bool all(Range auto&& range, auto&& pred) {
  for (auto&& x : range)
    if (!pred(x))
      return false;
  return true;
}
bool any(Range auto&& range, auto&& pred) {
  for (auto&& x : range)
    if (pred(x))
      return true;
  return false;
}
bool none(Range auto&& range, auto&& pred) {
  for (auto&& x : range)
    if (pred(x))
      return false;
  return true;
}

void iter_swap(auto a, auto b) {
  if (a != b)
    psl::swap(*a, *b);
}

auto remove_if(Range auto&& range, auto&& pred) {
  auto i = psl::begin(range);
  auto last = psl::end(range);
  auto tail = i;
  for (; i != last; ++i) {
    if (!pred(*i)) {
      if (i != tail)
        *(tail++) = psl::move(*i);
      else
        ++tail;
    }
  }

  return tail;
}

auto partition(Range auto&& range, auto&& pred) {
  auto i = psl::begin(range);
  auto last = psl::end(range);
  auto tail = i;
  for (; i != last; ++i)
    if (pred(*i))
      iter_swap(tail++, i);

  return tail;
}

// `comp` must be a strict ordering, i.e. comp(x, x) = false for all valid x
void sort(Range auto&& range, auto&& comp) {
  auto first = psl::begin(range);
  auto last = psl::end(range);
  if (first == last)
    return;
  if (psl::next(first) == last)
    return;

  auto pivot = psl::partition(range, [rhs = *first, &comp](auto&& lhs) { return comp(lhs, rhs); });

  psl::sort(psl::range(first, pivot), comp);
  ++pivot;
  psl::sort(psl::range(pivot, last), comp);
}

// void nth_element(Range auto&& range, ForwardIterator auto it, auto&& f) {
//   (void)it;
//   sort(range, f);
// }
// void partial_sort(Range auto&& range, ForwardIterator auto it [[maybe_unused]], auto&& f) {
//   sort(range, f);
// }

void reverse(Range auto&& range) {
  auto first = psl::begin(range);
  auto last = psl::end(range);
  if (first == last)
    return;
  --last;
  while (first < last) {
    swap(*first, *last);
    ++first;
    --last;
  }
}

namespace pipe {
auto operator|(Range auto&& range, auto&& adapter) {
  return adapter(range);
}
}  // namespace pipe

template <Range ARange, typename F>
auto transform(ARange&& range, F f) {
  struct Ranger {
    struct Iterator {
      decltype(auto) operator*() const {
        return f(*it);
      }
      Iterator& operator++() {
        ++it;
        return *this;
      }
      Iterator operator++(int) {
        auto old = *this;
        ++(*this);
        return old;
      }
      auto unwrap() const {
        return it;
      }
      bool operator==(const Iterator& b) const {
        return it == b.it;
      }
      bool operator!=(const Iterator& b) const {
        return !(*this == b);
      }

      F& f;
      IteratorTypeT<ARange> it;
    };
    Iterator begin() {
      return {f, psl::begin(range)};
    }
    Iterator end() {
      return {f, psl::end(range)};
    }
    size_t size() {
      return psl::size(range);
    }

    F f;
    ARange range;
  };
  return Ranger{psl::move(f), psl::forward<ARange>(range)};
}

auto transform_(auto&& f) {
  return [f](Range auto&& range) { return transform(FWD(range), f); };
}

template <Range ARange, typename F>
auto filter(ARange&& range, F f) {
  struct Ranger {
    struct Iterator {
      decltype(auto) operator*() const {
        return *it;
      }
      Iterator& operator++() {
        ++it;
        while (it != end && !f(*it))
          ++it;
        return *this;
      }
      Iterator operator++(int) {
        auto old = *this;
        ++(*this);
        return old;
      }
      auto unwrap() const {
        return it;
      }
      bool operator==(const Iterator& b) const {
        return it == b.it;
      }
      bool operator!=(const Iterator& b) const {
        return !(*this == b);
      }

      F& f;
      IteratorTypeT<ARange> it;
      IteratorTypeT<ARange> end;
    };
    Iterator begin() {
      auto it = psl::begin(range);
      auto last = psl::end(range);
      while (it != last && !f(*it))
        ++it;
      return {f, it, last};
    }
    Iterator end() {
      return {f, psl::end(range), psl::end(range)};
    }

    F f;
    ARange range;
  };
  return Ranger{psl::move(f), psl::forward<ARange>(range)};
}
auto filter_(auto&& f) {
  return [f](Range auto&& range) { return filter(FWD(range), f); };
}

template <Range ARange>
auto reverse_adapter(ARange&& range) {
  struct Ranger {
    struct Iterator {
      decltype(auto) operator*() const {
        return *rit();
      }
      Iterator& operator++() {
        ++it;
        return *this;
      }
      Iterator operator++(int) {
        auto old = *this;
        ++(*this);
        return old;
      }
      auto unwrap() const {
        if (it == end)
          return end;
        else
          return rit();
      }
      auto operator->() {
        return &(*rit());
      }
      bool operator==(const Iterator& b) const {
        return it == b.it;
      }
      bool operator!=(const Iterator& b) const {
        return !(*this == b);
      }

      IteratorTypeT<ARange> rit() const {
        return begin + (end - it - 1);
      }

      IteratorTypeT<ARange> it;
      IteratorTypeT<ARange> begin;
      IteratorTypeT<ARange> end;
    };
    Iterator begin() {
      return {psl::begin(range), psl::begin(range), psl::end(range)};
    }
    Iterator end() {
      return {psl::end(range), psl::begin(range), psl::end(range)};
    }
    size_t size() {
      return psl::size(range);
    }
    ARange range;
  };
  return Ranger{psl::forward<ARange>(range)};
}
inline auto reverse_() {
  return [](Range auto&& range) { return reverse_adapter(FWD(range)); };
}

template <Range ARange>
auto indirection(ARange&& range) {
  struct Ranger {
    struct Iterator {
      decltype(auto) operator*() const {
        return *(*it);
      }
      Iterator& operator++() {
        ++it;
        return *this;
      }
      Iterator operator++(int) {
        auto old = *this;
        ++it;
        return old;
      }
      auto operator->() {
        return it;
      }
      bool operator==(const Iterator& b) const {
        return it == b.it;
      }
      bool operator!=(const Iterator& b) const {
        return !(*this == b);
      }

      IteratorTypeT<ARange> it;
    };
    Iterator begin() {
      return {psl::begin(range)};
    }
    Iterator end() {
      return {psl::end(range)};
    }
    size_t size() {
      return psl::size(range);
    }
    ARange range;
  };
  return Ranger{psl::forward<ARange>(range)};
}
inline auto indirection_() {
  return [](Range auto&& range) { return indirection(FWD(range)); };
}

template <typename T, Range ARange>
T to(ARange&& range) {
  return T(psl::forward<ARange>(range));
}

template <typename T>
auto to_() {
  return [](Range auto&& range) { return to<T>(FWD(range)); };
}

auto trim(Range auto&& range, size_t offset, size_t len) {
  return psl::range(psl::begin(range) + offset, psl::begin(range) + offset + len);
}
inline auto trim_(size_t offset, size_t len) {
  return [offset, len](Range auto&& range) { return trim(FWD(range), offset, len); };
}

auto clip(Range auto&& range, RandomAccessIterator auto last) {
  return psl::range(psl::begin(range), last);
}
inline auto clip_(RandomAccessIterator auto last) {
  return [last](Range auto&& range) { return clip(FWD(range), last); };
}

template <typename ResultType>
auto sum(Range auto&& range) {
  auto s = ResultType();
  for (auto&& x : range)
    s += x;
  return s;
}
template <typename ResultType>
auto mean(Range auto&& range) {
  return sum<ResultType>(range) / psl::size(range);
}
template <typename ResultType>
auto variance(Range auto&& range) {
  auto var = ResultType();
  auto mean = psl::mean<ResultType>(range);
  for (auto&& x : range)
    var += (x - mean) * (x - mean);
  return var / psl::size(range);
}

auto find_last_of(Range auto&& range, const auto& value) {
  return find_if(psl::reverse_adapter(range), equal_to(value)).unwrap();
}

template <Range RangeA, Range RangeB>
auto tie_adapter(RangeA&& a, RangeB&& b) {
  struct Ranger {
    struct Iterator {
      decltype(auto) operator*() {
        decltype(auto) a = *a_it;
        decltype(auto) b = *b_it;
        return psl::pair<decltype(a), decltype(b)>{FWD(a), FWD(b)};
      }
      decltype(auto) operator*() const {
        decltype(auto) a = *a_it;
        decltype(auto) b = *b_it;
        return psl::pair<decltype(a), decltype(b)>{FWD(a), FWD(b)};
      }
      Iterator& operator++() {
        ++a_it;
        ++b_it;
        return *this;
      }
      Iterator operator++(int) {
        auto old = *this;
        ++(*this);
        return old;
      }
      bool operator==(const Iterator& b) const {
        return a_it == b.a_it || b_it == b.b_it;
      }
      bool operator!=(const Iterator& b) const {
        return !(*this == b);
      }

      IteratorTypeT<RangeA> a_it;
      IteratorTypeT<RangeB> b_it;
    };
    Iterator begin() {
      return {psl::begin(a), psl::begin(b)};
    }
    Iterator end() {
      return {psl::end(a), psl::end(b)};
    }

    RangeA a;
    RangeB b;
  };
  return Ranger{FWD(a), FWD(b)};
}

}  // namespace psl
