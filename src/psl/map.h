#pragma once
#include <psl/algorithm.h>

#include <map>

namespace psl {

template <typename Key, typename Value, typename Pred = less<>>
struct map : std::map<Key, Value, Pred> {
  using value_type = Value;
};

template <typename Key, typename Value, typename Pred = less<>>
struct multimap : std::multimap<Key, Value, Pred> {
  using value_type = Value;
};

}  // namespace psl
