#pragma once

#include <psl/vector.h>

#include <map>

namespace psl {

template <typename Key, typename Value, typename Pred = less<>>
struct map : std::map<Key, Value, Pred> {
};

template <typename Key, typename Value, typename Pred = less<>>
struct multimap : std::multimap<Key, Value, Pred> {};

auto find_or(auto&& map, auto&& value, auto fallback) {
  if (auto it = map.find(value); it != map.end())
    return it->second;
  else
    return fallback;
}

}  // namespace psl
