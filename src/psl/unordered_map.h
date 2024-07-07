#pragma once
#include <unordered_map>

namespace psl {

template <typename Key, typename Value>
struct unordered_map : std::unordered_map<Key, Value> {
  using value_type = Value;
};

template <typename Key, typename Value>
struct unordered_multimap : std::unordered_multimap<Key, Value> {
  using value_type = Value;
};

}  // namespace psl
