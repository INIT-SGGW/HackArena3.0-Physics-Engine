#pragma once

#include <tuple>

template <template <typename...> class T, typename Tuple>
struct apply_tuple;

template <template <typename...> class T, typename... Args>
struct apply_tuple<T, std::tuple<Args...>> { using type = T<Args...>; };

template <template <typename...> class T, typename Tuple>
using apply_tuple_t = typename apply_tuple<T, Tuple>::type;
