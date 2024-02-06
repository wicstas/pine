#pragma once

#include <psl/stdint.h>

namespace psl {

template <typename T, size_t N>
struct Array;

template <typename Key, typename Value, typename Pred>
struct map;
template <typename Key, typename Value, typename Pred>
struct multimap;

template <typename T>
struct default_deleter;
template <typename T, typename Deleter>
class unique_ptr;
template <typename T>
class shared_ptr;

template <typename T>
struct optional;

template <typename T>
struct span;

class string;
class string_view;

template <typename Key, typename Value>
struct unordered_map;
template <typename Key, typename Value>
struct unordered_multimap;

template <typename... Ts>
struct variant;

template <typename T>
struct default_allocator;
template <typename T, size_t capacity>
struct static_allocator;

template <typename T, typename Allocator>
class vector;
template <typename T, size_t capacity>
struct static_vector;

template <typename... Us, typename... Ts>
auto vector_of(Ts... xs);

}  // namespace psl
