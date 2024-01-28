#pragma once

#include <psl/vector.h>

#include <unordered_map>

namespace psl {

template <typename Key, typename Value>
using unordered_map = std::unordered_map<Key, Value>;

template <typename Key, typename Value>
using unordered_multimap = std::unordered_multimap<Key, Value>;

}  // namespace psl
