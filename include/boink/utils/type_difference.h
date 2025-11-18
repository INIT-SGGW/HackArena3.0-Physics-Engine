#pragma once

#include <utility>
#include <type_traits>
#include <tuple>

namespace boink
{
  template <typename T, typename... List>
  struct contains_type : std::false_type {};

  template <typename T, typename Head, typename... Tail>
  struct contains_type<T, Head, Tail...>
      : contains_type<T, Tail...> {};

  template <typename T, typename... Tail>
  struct contains_type<T, T, Tail...>
      : std::true_type {};

  template <typename TupleT_,typename TupleFull_>
  struct type_difference;


  template <typename... Ts_,typename... Us_>
  struct type_difference<std::tuple<Ts_...>,std::tuple<Us_...>>
  {
  private:
    template<typename U>
    using R=std::conditional_t<contains_type<U,Ts_...>::value,std::tuple<>,std::tuple<U>>;

  public:
    /*
    This works like set diffrenece (T_Full\T_Exists) given:
    using T_Exists=std::tuple<int,unsigned int,char>;
    using T_Full=std::tuple<float,int,double,char>;

    The resulting type will be: 
    std::tuple<float,double>;
    */
    using type=decltype(std::tuple_cat(std::declval<R<Us_>>()...));
  };
}
