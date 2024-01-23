#pragma once

#include <pine/psl/type_traits.h>
#include <pine/psl/utility.h>
#include <algorithm>

namespace psl {

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

template <typename It>
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
    while (n--)
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
    while (n--)
      --it;
    return it;
  }
}

template <OutputIterator OutputIt, Range Input>
void copy(OutputIt d_first, Input&& input) {
  auto first = psl::begin(input);
  auto last = psl::end(input);
  for (; first != last; ++first, ++d_first)
    *d_first = *first;
}

template <OutputIterator OutputIt, Range Input>
void move(OutputIt d_first, Input&& input) {
  auto first = psl::begin(input);
  auto last = psl::end(input);
  for (; first != last; ++first, ++d_first)
    *d_first = psl::move(*first);
}

template <Range Output, typename T>
void fill(Output&& output, const T& value) {
  auto first = psl::begin(output);
  auto last = psl::end(output);
  for (; first != last; ++first)
    *first = value;
}

template <typename T = void>
struct less {
  bool operator()(const T& l, const T& r) const {
    return l < r;
  }
};
template <>
struct less<void> {
  using is_transparent = void;
  bool operator()(const auto& l, const auto& r) const {
    return l < r;
  }
};

template <typename T>
struct less_than {
  less_than(T value) : value(psl::move(value)) {
  }
  bool operator()(const auto& x) const {
    return x < value;
  }
  T value;
};

// Return the iterator to the first element not satisfying `pred`
template <Range ARange, typename Pred>
auto lower_bound(ARange&& range, Pred&& pred) {
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

template <Range ARange, typename F>
auto find_if(ARange&& range, F f) {
  auto first = psl::begin(range);
  auto last = psl::end(range);
  for (; first != last; ++first)
    if (f(*first))
      return first;
  return last;
}

template <Range ARange, typename T>
auto find(ARange&& range, const T& value) {
  return psl::find_if(range, [&value](const auto& x) { return x == value; });
}

template <Range ARange, typename T>
bool has(ARange&& range, const T& value) {
  return psl::find(range, value) != psl::end(range);
}

template <Range ARange, typename T>
auto find_next(ARange&& range, const T& value, size_t n) {
  auto it = psl::begin(range);
  auto end = psl::end(range);
  for (; n != 0; n--) {
    it = find(psl::range(it, end), value);
    if (it == end)
      break;
    else
      ++it;
  }
  return it;
}

// template <Range ARange, typename T>
// void insert_at(ARange&& range, const T& pivot, const T& value) {
//   auto pos = size_t{0};
//   while (true) {
//     auto it = psl::find(psl::range(psl::begin(range) + pos, psl::end(range)), pivot);
//     if (it == psl::end(range))
//       return;
//     pos = it - psl::begin(range);
//   }
// }

template <Range ARange, typename T>
auto find_last_of(ARange&& range, const T& value) {
  auto first = psl::begin(range);
  auto last = psl::prev(psl::end(range));
  for (; last != first; --last)
    if (*last == value)
      return last;
  return first;
}

template <Range ARange, typename T>
void replace(ARange&& range, const T& old, const T& new_) {
  auto first = psl::begin(range);
  auto last = psl::end(range);
  for (; first != last; ++first)
    if (*first == old)
      *first = new_;
}

template <Range ARange, typename T>
size_t count(ARange&& range, const T& value) {
  auto c = size_t{0};
  auto first = psl::begin(range);
  auto last = psl::end(range);
  for (; first != last; ++first)
    if (*first == value)
      ++c;
  return c;
}

// template <Range ARange, typename F>
// auto remove_if(ARange&& range, F&& f) {
//   auto first = psl::begin(range);
//   auto last = psl::end(range);
//   auto tail = first;
//   for (; first != last; ++first)
//     if (first != tail && !f(*first))
//       *(tail++) = *first;

//   return tail;
// }

template <Range ARange, typename Pred>
void sort(ARange&& range, Pred&& pred) {
  auto first = psl::begin(range);
  auto last = psl::end(range);

  if (first == last)
    return;
  auto pivot = first;

  auto i = first;
  ++i;
  if (i == last)
    return;

  for (; i != last; ++i) {
    if (pred(*i, *pivot)) {
      auto prev = pivot;
      ++pivot;

      psl::swap(*prev, *i);
      if (pivot != i)
        psl::swap(*pivot, *i);
    }
  }

  psl::sort(psl::range(first, pivot), pred);
  ++pivot;
  psl::sort(psl::range(pivot, last), pred);
}

template <Range ARange, typename Pred>
auto partition(ARange&& range, Pred&& pred) {
  auto head = psl::begin(range);
  auto last = psl::end(range);
  return std::partition(head, last, pred);
  // auto tail = head;
  // for (; head != last; ++head) {
  //   if (pred(*head)) {
  //     psl::swap(*(tail++), *head);
  //   }
  // }

  // return tail;
}

template <Range ARange, ForwardIterator It, typename F>
void nth_element(ARange&& range, It it, F&& f) {
  return std::nth_element(psl::begin(range), it, psl::end(range), f);
  // sort(range, f);
  // auto value = *it;
  // return partition(range, [&](auto&& rhs) { return f(rhs, value); });
}
template <Range ARange, ForwardIterator It, typename F>
void partial_sort(ARange&& range, It it, F&& f) {
  return std::partial_sort(psl::begin(range), psl::end(range), it, f);
  // sort(range, f);
}

template <Range ARange>
void reverse(ARange&& range) {
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
        auto copy = *this;
        ++it;
        return copy;
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

template <Range ARange, typename F>
auto filter(ARange&& range, F f) {
  struct Ranger {
    struct Iterator {
      decltype(auto) operator*() const {
        return;
      }
      Iterator& operator++() {
        ++it;
        while (!f(*it))
          ++it;
        return *this;
      }
      Iterator operator++(int) {
        auto copy = *this;
        ++copy;
        return copy;
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
      while (!f(*it))
        ++it;
      return {f, it, psl::end(range)};
    }
    Iterator end() {
      return {f, psl::end(range), psl::end(range)};
    }
    size_t size() {
      return psl::size(range);
    }
    F f;
    ARange range;
  };
  return Ranger{psl::move(f), psl::forward<ARange>(range)};
}

template <Range ARange>
auto reverse_ranger(ARange&& range) {
  struct Ranger {
    struct Iterator {
      decltype(auto) operator*() const {
        return *rit();
      }
      Iterator& operator++() {
        --it;
        return *this;
      }
      Iterator operator++(int) {
        auto copy = *this;
        --it;
        return copy;
      }
      operator IteratorTypeT<ARange>() const {
        return it;
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

      auto rit() const {
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

template <typename T, Range ARange>
T to(ARange&& range) {
  return T{psl::forward<ARange>(range)};
}

template <Range ARange>
auto trim(ARange&& range, size_t a, size_t b) {
  return decltype(range){psl::begin(range) + a, psl::begin(range) + b};
}

template <Range ARange>
auto sum(ARange&& range) {
  auto s = psl::Decay<decltype(*psl::begin(range))>{0};
  for (auto&& x : range)
    s += x;
  return s;
}

}  // namespace psl
