#pragma once

#include <pine/psl/stdint.h>

namespace psl {

template <typename T, T Value>
struct IntegralConstant {
  using ValueType = T;

  static constexpr T value = Value;
};

using TrueType = IntegralConstant<bool, true>;
using FalseType = IntegralConstant<bool, false>;

template <typename T>
struct TypeIdentity {
  using Type = T;
};
template <typename T>
using TypeIdentityT = typename TypeIdentity<T>::Type;

template <typename... Ts>
using VoidT = void;

template <bool, typename T = void>
struct EnableIf {};
template <typename T>
struct EnableIf<true, T> {
  using Type = T;
};
template <bool Value, typename T = void>
using EnableIfT = typename EnableIf<Value, T>::Type;

template <bool, typename T, typename U>
struct Conditional;
template <typename T, typename U>
struct Conditional<false, T, U> {
  using type = T;
};
template <typename T, typename U>
struct Conditional<true, T, U> {
  using type = U;
};
template <bool Value, typename T, typename U>
using ConditionalT = typename Conditional<Value, T, U>::type;

template <typename T, typename U>
struct _SameAs : FalseType {};
template <typename T>
struct _SameAs<T, T> : TrueType {};
template <typename T, typename U>
concept SameAs = _SameAs<T, U>::value;

template <typename T>
concept IsVoid = SameAs<T, void>;

template <typename T, typename U>
concept DifferentFrom = !SameAs<T, U>;

template <typename T>
struct IsArray : FalseType {};
template <typename T>
struct IsArray<T[]> : TrueType {};
template <typename T, int N>
struct IsArray<T[N]> : TrueType {};
template <typename T>
constexpr bool isArray = IsArray<T>::value;

template <typename T>
struct IsFunction : FalseType {};
template <typename R, typename... Args>
struct IsFunction<R(Args...)> : TrueType {};
template <typename T>
constexpr bool isFunction = IsFunction<T>::value;

template <typename T>
struct IsPointer : FalseType {};
template <typename T>
struct IsPointer<T *> : TrueType {};
template <typename T>
constexpr bool is_pointer = IsPointer<T>::value;

template <typename T>
struct IsIntegral : FalseType {};
template <>
struct IsIntegral<int8_t> : TrueType {};
template <>
struct IsIntegral<int16_t> : TrueType {};
template <>
struct IsIntegral<int32_t> : TrueType {};
template <>
struct IsIntegral<int64_t> : TrueType {};
template <>
struct IsIntegral<uint8_t> : TrueType {};
template <>
struct IsIntegral<uint16_t> : TrueType {};
template <>
struct IsIntegral<uint32_t> : TrueType {};
template <>
struct IsIntegral<uint64_t> : TrueType {};
template <typename T>
constexpr bool isIntegral = IsIntegral<T>::value;
template <typename T>
concept Integral = isIntegral<T>;

template <typename T>
struct IsFloatingPoint : FalseType {};
template <>
struct IsFloatingPoint<float> : TrueType {};
template <>
struct IsFloatingPoint<double> : TrueType {};
template <typename T>
constexpr bool is_floating_point = IsFloatingPoint<T>::value;
template <typename T>
concept FloatingPoint = is_floating_point<T>;
template <typename T>
concept FundamentalArithmetic = Integral<T> || FloatingPoint<T>;
template <typename T>
concept FundamentalType = FundamentalArithmetic<T> || isArray<T> || is_pointer<T> || isFunction<T>;

template <typename T>
concept Arithmetic = requires(T x) {
  x *x;
  x / x;
  x + x;
  x - x;
  x += x;
  x -= x;
};
template <typename T>
concept LinearArithmetic = requires(T x) {
  x + x;
  x - x;
  x += x;
  x -= x;
};

template <typename T>
struct IsReference : FalseType {};
template <typename T>
struct IsReference<T &> : TrueType {};
template <typename T>
constexpr bool is_reference = IsReference<T>::value;

template <typename T>
struct IsRvReference : FalseType {};
template <typename T>
struct IsRvReference<T &&> : TrueType {};
template <typename T>
constexpr bool is_rv_reference = IsRvReference<T>::value;

template <typename T>
struct IsConst : FalseType {};
template <typename T>
struct IsConst<const T> : TrueType {};
template <typename T>
constexpr bool is_const = IsConst<T>::value;

template <typename T>
struct RemoveConst {
  using type = T;
};
template <typename T>
struct RemoveConst<const T> {
  using type = T;
};
template <typename T>
using RemoveConstT = typename RemoveConst<T>::type;

template <typename T>
struct RemoveReference {
  using type = T;
};
template <typename T>
struct RemoveReference<T &> {
  using type = T;
};
template <typename T>
struct RemoveReference<T &&> {
  using type = T;
};
template <typename T>
using RemoveReferenceT = typename RemoveReference<T>::type;

template <typename T>
struct RemoveExtent {
  using type = T;
};
template <typename T>
struct RemoveExtent<T[]> {
  using type = T;
};
template <typename T, size_t N>
struct RemoveExtent<T[N]> {
  using type = T;
};
template <typename T>
using RemoveExtentT = typename RemoveExtent<T>::type;

template <typename T, bool IsArray = isArray<T>, bool IsFunction = isFunction<T>>
struct DecaySelector;
template <typename T>
struct DecaySelector<T, false, false> {
  using Type = RemoveConstT<T>;
};
template <typename T>
struct DecaySelector<T, true, false> {
  using Type = RemoveExtentT<T> *;
};
template <typename T>
struct DecaySelector<T, false, true> {
  using Type = T *;
};
template <typename T>
struct Decay {
private:
  using NoRef = RemoveReferenceT<T>;

public:
  using Type = typename DecaySelector<NoRef>::Type;
};
template <typename T>
using DecayT = typename Decay<T>::Type;

template <typename T>
T declval() {
  return declval<T>();
}

template <typename F, typename... Args>
using ReturnTypeT = decltype(psl::declval<F>()(psl::declval<Args>()...));

template <typename From, typename To>
struct IsConverible {
private:
  static constexpr TrueType check(To);
  static constexpr FalseType check(...);

public:
  static constexpr bool value = decltype(check(psl::declval<From>()))::value;
};
template <typename From, typename To>
constexpr bool is_convertible = IsConverible<From, To>::value;

template <typename T>
struct CorrespondingInt;
template <>
struct CorrespondingInt<float> {
  using type = int32_t;
};
template <>
struct CorrespondingInt<double> {
  using type = int64_t;
};
template <typename T>
using CorrespondingIntT = typename CorrespondingInt<T>::type;

template <typename T>
struct CorrespondingUint;
template <>
struct CorrespondingUint<float> {
  using type = uint32_t;
};
template <>
struct CorrespondingUint<double> {
  using type = uint64_t;
};
template <typename T>
using CorrespondingUintT = typename CorrespondingUint<T>::type;

template <typename T, typename = void>
struct IsDereferenceable {
  static constexpr bool value = false;
};
template <typename T>
struct IsDereferenceable<T, VoidT<decltype(*psl::declval<T>())>> {
  static constexpr bool value = true;
};
template <typename T>
constexpr bool is_dereferenceable = IsDereferenceable<T>::value;

template <bool value, typename... Ts>
constexpr bool deferred_bool = value;

template <int I>
struct PriorityTag : PriorityTag<I - 1> {};
template <>
struct PriorityTag<0> {};

template <typename I, I...>
struct IntegerSequence {};

template <typename I, int N>
struct MakeIntegerSequenceImpl;
template <typename I>
struct MakeIntegerSequenceImpl<I, 0> : IntegerSequence<I> {};
template <typename I>
struct MakeIntegerSequenceImpl<I, 1> : IntegerSequence<I, 0> {};
template <typename I>
struct MakeIntegerSequenceImpl<I, 2> : IntegerSequence<I, 0, 1> {};
template <typename I>
struct MakeIntegerSequenceImpl<I, 3> : IntegerSequence<I, 0, 1, 2> {};
template <typename I>
struct MakeIntegerSequenceImpl<I, 4> : IntegerSequence<I, 0, 1, 2, 3> {};
template <typename I>
struct MakeIntegerSequenceImpl<I, 5> : IntegerSequence<I, 0, 1, 2, 3, 4> {};
template <typename I>
struct MakeIntegerSequenceImpl<I, 6> : IntegerSequence<I, 0, 1, 2, 3, 4, 5> {};
template <typename I>
struct MakeIntegerSequenceImpl<I, 7> : IntegerSequence<I, 0, 1, 2, 3, 4, 5, 6> {};
template <typename I>
struct MakeIntegerSequenceImpl<I, 8> : IntegerSequence<I, 0, 1, 2, 3, 4, 5, 6, 7> {};
template <typename I>
struct MakeIntegerSequenceImpl<I, 9> : IntegerSequence<I, 0, 1, 2, 3, 4, 5, 6, 7, 8> {};
template <typename I>
struct MakeIntegerSequenceImpl<I, 10> : IntegerSequence<I, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9> {};
template <typename I>
struct MakeIntegerSequenceImpl<I, 11> : IntegerSequence<I, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10> {};
template <typename I>
struct MakeIntegerSequenceImpl<I, 12> : IntegerSequence<I, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11> {};

template <typename I, int N>
auto make_integer_sequence() {
  return MakeIntegerSequenceImpl<I, N>{};
}

template <typename T, int I>
struct IndexedType {
  using Type = T;
  static constexpr int index = I;
};
template <typename... Ts>
struct IndexedTypeSequence {};

template <size_t I, typename Seq, typename... Ts>
struct MakeIndexedTypeSequenceHelper;
template <size_t I, typename... Ps, typename T, typename... Ts>
struct MakeIndexedTypeSequenceHelper<I, IndexedTypeSequence<Ps...>, T, Ts...>
    : MakeIndexedTypeSequenceHelper<I + 1, IndexedTypeSequence<Ps..., IndexedType<T, I>>, Ts...> {};
template <size_t I, typename... Ps>
struct MakeIndexedTypeSequenceHelper<I, IndexedTypeSequence<Ps...>> : IndexedTypeSequence<Ps...> {};
template <typename... Ts>
auto make_indexed_type_sequence() {
  return IndexedTypeSequence{MakeIndexedTypeSequenceHelper<0, IndexedTypeSequence<>, Ts...>{}};
};

template <typename T, typename... Ts>
struct FirstType {
  using Type = T;
};
template <typename... Ts>
using FirstTypeT = typename FirstType<Ts...>::Type;

template <int I, typename T, typename... Ts>
struct NthType : NthType<I - 1, Ts...> {};
template <typename T, typename... Ts>
struct NthType<0, T, Ts...> {
  using Type = T;
};
template <int I, typename... Ts>
using NthTypeT = typename NthType<I, Ts...>::Type;

template <typename... Ts>
struct TypePack {};

template <typename T, typename TP>
constexpr bool one_of = false;
template <typename T, typename... Ts>
constexpr bool one_of<T, TypePack<Ts...>> = (SameAs<T, Ts> || ...);

template <template <typename...> typename R, typename T>
struct CopyTemplate;
template <template <typename...> typename R, typename... Ts>
struct CopyTemplate<R, TypePack<Ts...>> {
  using Type = R<Ts...>;
};
template <template <typename...> typename R, typename T>
using CopyTemplateT = typename CopyTemplate<R, T>::Type;

template <typename T, typename U, char C>
struct OpResult;
template <typename T, typename U, char C>
using OpResultT = typename OpResult<T, U, C>::type;
template <typename T, typename U>
struct OpResult<T, U, '+'> {
  using type = decltype(declval<T>() + declval<U>());
};
template <typename T, typename U>
struct OpResult<T, U, '-'> {
  using type = decltype(declval<T>() - declval<U>());
};
template <typename T, typename U>
struct OpResult<T, U, '*'> {
  using type = decltype(declval<T>() * declval<U>());
};
template <typename T, typename U>
struct OpResult<T, U, '/'> {
  using type = decltype(declval<T>() / declval<U>());
};
template <typename T, typename U>
struct OpResult<T, U, '%'> {
  using type = decltype(declval<T>() & declval<U>());
};

template <typename T>
struct MpClassType;
template <typename T>
using MpClassTypeT = typename MpClassType<T>::Type;
template <typename T, typename R, typename... Args>
struct MpClassType<R (T::*)(Args...)> {
  using Type = T;
};
template <typename T>
struct MpReturnType;
template <typename T>
using MpReturnTypeT = typename MpReturnType<T>::Type;
template <typename T, typename R, typename... Args>
struct MpReturnType<R (T::*)(Args...)> {
  using Type = R;
};

template <typename... Ts>
constexpr bool same_type = false;
template <>
inline constexpr bool same_type<> = true;
template <typename T>
constexpr bool same_type<T> = true;
template <typename T, typename... Ts>
constexpr bool same_type<T, T, Ts...> = same_type<T, Ts...>;

template <typename T>
concept DefaultConstructible = requires { T(); };
template <typename T>
concept Destructible = requires(T x) { x.~T(); };
template <typename T>
concept MoveConstructible = requires(T x) { T(static_cast<T &&>(x)); };
template <typename T>
concept CopyConstructible = MoveConstructible<T> && requires(T x) { T(x); };
template <typename T>
concept MoveAssignable = requires(T x) { x = static_cast<T &&>(x); };
template <typename T>
concept CopyAssignable = MoveAssignable<T> && requires(T x) { x = x; };
template <typename T>
concept Movable = MoveConstructible<T> && MoveAssignable<T>;
template <typename T>
concept Copyable = CopyConstructible<T> && CopyAssignable<T>;
template <typename T, typename U>
concept EqualityComparable = requires(T a, U b) {
  { a == b } -> SameAs<bool>;
  { a != b } -> SameAs<bool>;
};
template <typename T, typename U>
concept Comparable = requires(T a, U b) {
  { a == b } -> SameAs<bool>;
  { a != b } -> SameAs<bool>;
  { a < b } -> SameAs<bool>;
  { a > b } -> SameAs<bool>;
  { a <= b } -> SameAs<bool>;
  { a >= b } -> SameAs<bool>;
};
template <typename T>
concept Semiregular = Copyable<T> && DefaultConstructible<T>;
template <typename T>
concept Regular = Semiregular<T> && EqualityComparable<T, T>;

template <typename T>
struct IteratorTraits;
template <typename T>
using IteratorValueType = typename IteratorTraits<T>::ValueType;
template <typename T>
using IteratorReferenceType = typename IteratorTraits<T>::ReferenceType;
template <typename T>
using IteratorDifferenceType = typename IteratorTraits<T>::DifferenceType;

template <typename T>
struct IteratorTraits<T *> {
  using ValueType = T;
  using ReferenceType = T &;
  using DifferenceType = ptrdiff_t;
};

template <typename T>
requires requires {
  typename T::ValueType;
  typename T::ReferenceType;
  typename T::DifferenceType;
}
struct IteratorTraits<T> {
  using ValueType = typename T::ValueType;
  using ReferenceType = typename T::ReferenceType;
  using DifferenceType = typename T::DifferenceType;
};

template <typename T>
concept ForwardIterator =
    Copyable<T> && is_dereferenceable<T> && EqualityComparable<T, T> && requires(T it) {
      typename IteratorValueType<T>;
      { ++it } -> SameAs<T &>;
      it++;
    };
template <typename T>
concept BackwardIterator =
    Copyable<T> && is_dereferenceable<T> && EqualityComparable<T, T> && requires(T it) {
      typename IteratorValueType<T>;
      { --it } -> SameAs<T &>;
      it--;
    };
template <typename T>
concept BidirectionalIterator =
    Copyable<T> && is_dereferenceable<T> && EqualityComparable<T, T> && requires(T it) {
      typename IteratorValueType<T>;
      { ++it } -> SameAs<T &>;
      { --it } -> SameAs<T &>;
      it++;
      it--;
    };
template <typename T>
concept OutputIterator =
    ForwardIterator<T> && requires(T it) { *it = declval<IteratorValueType<T>>(); };
template <typename T>
concept RandomAccessIterator =
    BidirectionalIterator<T> && Arithmetic<T> && requires(T it, IteratorDifferenceType<T> n) {
      { it[n] } -> SameAs<IteratorReferenceType<T>>;
    };

template <typename Container, typename T>
struct ChangeBasis {
  using Type = typename Container::template ChangeBasis<T>;
};
template <typename R, typename U, typename T>
struct ChangeBasis<R (*)(U), T> {
  using Type = R (*)(T);
};
template <typename Container, typename T>
using ChangeBasisT = typename ChangeBasis<Container, T>::Type;

}  // namespace psl
