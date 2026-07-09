/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Guo Zhichong <guozhichong@sensetime.com>
 */

#pragma once

#include <iostream>
#include <tuple>        // std::tuple
#include <memory>       // std::unique_ptr/std::shared_ptr/std::weak_ptr
#include <type_traits>  // std::remove_reference/std::remove_cv
#include <utility>      // std::forward

namespace senseAD {
namespace perception {
namespace type_traits {

template <bool, typename T = void>
struct EnableIf {};

template <typename T>
struct EnableIf<true, T> {
    typedef T type;
};

template <bool, typename TypeIfTrue, typename TypeIfFalse>
struct EnableIfElse {};

template <typename TypeIfTrue, typename TypeIfFalse>
struct EnableIfElse<false, TypeIfTrue, TypeIfFalse> {
    typedef TypeIfFalse type;
};

template <typename TypeIfTrue, typename TypeIfFalse>
struct EnableIfElse<true, TypeIfTrue, TypeIfFalse> {
    typedef TypeIfTrue type;
};

template <typename T>
struct RemovePointer {
    typedef T type;
};

template <typename T>
struct RemovePointer<T *> : RemovePointer<T> {};

template <typename T>
struct RemovePointer<std::shared_ptr<T>> : RemovePointer<T> {};

template <typename T>
struct RemovePointer<std::unique_ptr<T>> : RemovePointer<T> {};

template <typename T>
struct RemovePointer<std::weak_ptr<T>> : RemovePointer<T> {};

template <typename T>
struct RemovePointer<const T> : RemovePointer<T> {};

template <typename T>
struct RemovePointer<T &> : RemovePointer<T> {};

template <typename T>
struct RemovePointer<T &&> : RemovePointer<T> {};

template <typename T>
using RemoveCV = std::remove_cv<T>;
template <typename T>
using RemoveCV_t = typename RemoveCV<T>::type;
template <typename T>
using RemoveRef = std::remove_reference<T>;
template <typename T>
using RemoveRef_t = typename RemoveRef<T>::type;
template <typename T>
using RemoveCVRef = RemoveCV<RemoveRef_t<T>>;
template <typename T>
using RemoveCVRef_t = RemoveCV_t<RemoveRef_t<T>>;

template <typename T>
struct IsTuple : std::false_type {};
template <typename... Ts>
struct IsTuple<std::tuple<Ts...>> : std::true_type {};
template <typename T>
struct IsTuple<const T> : IsTuple<T> {};
template <typename T>
struct IsTuple<T &> : IsTuple<T> {};

template <int... Is>
struct Seq {};

template <int N, int... Is>
struct GenerateSeq : GenerateSeq<N - 1, N - 1, Is...> {};

template <int... Is>
struct GenerateSeq<0, Is...> : Seq<Is...> {};

// C++11-style
// Apply tuple on function
template <std::size_t N>
struct ApplyTuple {
    template <typename Func, typename... Ts, typename... Us>
    static void Go(Func &&func, std::tuple<Ts...> &tuple_args, Us &&... args) {  // NOLINT
        ApplyTuple<N - 1>::Go(std::forward<Func>(func), tuple_args,
                              std::get<N - 1>(tuple_args),
                              std::forward<Us>(args)...);
    }
};

template <>
struct ApplyTuple<0> {
    template <typename Func, typename... Ts, typename... Us>
    static void Go(Func &&func, std::tuple<Ts...> &tuple_args, Us &&... args) {  // NOLINT
        func(std::forward<Us>(args)...);
    }
};

template <bool unpack,
          std::size_t N,
          template <class> class Cond,
          typename T,
          typename... Ts>
struct ApplyTupleType
    : ApplyTupleType<unpack, N - 1, Cond, Ts..., typename Cond<T>::type> {};

template <template <class> class Cond, typename T, typename... Ts>
struct ApplyTupleType<false, 0, Cond, T, Ts...> {
    using type = std::tuple<T, Ts...>;
};

template <std::size_t N, template <class> class Cond, typename... Ts>
struct ApplyTupleType<true, N, Cond, std::tuple<Ts...>>
    : ApplyTupleType<false, N, Cond, Ts...> {};

template <bool append, typename... Ts>
struct JoinTupleType;

template <typename... Ts, typename... Us>
struct JoinTupleType<true, std::tuple<Ts...>, Us...> {
    using type = std::tuple<Ts..., Us...>;
};

template <typename... Ts, typename... Us>
struct JoinTupleType<false, std::tuple<Ts...>, Us...> {
    using type = std::tuple<Us..., Ts...>;
};

}  // namespace type_traits
}  // namespace perception
}  // namespace senseAD
