#pragma once

#include <psl/vector.h>

#include <map>

namespace psl {

template <typename Key, typename Value, typename Pred = less<>>
using map = std::map<Key, Value, Pred>;

template <typename Key, typename Value, typename Pred = less<>>
using multimap = std::multimap<Key, Value, Pred>;

}  // namespace psl
