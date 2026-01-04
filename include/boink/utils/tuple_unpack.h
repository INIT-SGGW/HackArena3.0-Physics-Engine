#pragma once

#include <tuple>
#include <type_traits>

template <template <typename...> class T, typename Tuple>
struct apply_tuple;

template <template <typename...> class T, typename... Args>
struct apply_tuple<T, std::tuple<Args...>> { using type = T<Args...>; };

template <template <typename...> class T, typename Tuple>
using apply_tuple_t = typename apply_tuple<T, Tuple>::type;

template<typename Target,typename First, typename... Rest>
decltype(auto) getComponent(First&& first, Rest&&... rest)
{
  using TargetBase=std::remove_cvref_t<Target>;
  using FirstBase=std::remove_cvref_t<First>;
  if constexpr(std::is_same_v<TargetBase,FirstBase>)
  {
    return std::forward<First>(first);
  }
  else
  {
    static_assert(sizeof...(Rest)>0,"Component not found");
    return getComponent<Target>(std::forward<Rest>(rest)...);
  }
}
