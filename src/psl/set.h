#pragma once
#include <psl/algorithm.h>

#include <set>

namespace psl {

template <typename Value, typename Pred = less<>>
struct set : std::set<Value, Pred> {
};

}  // namespace psl
