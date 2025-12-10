#include <iostream>

#include "boink/utils/type_difference.h"

using namespace boink;

int main() {
  using T_Exists = std::tuple<int, unsigned int, char>;
  using T_Full = std::tuple<float, int, double, char>;

  // It works like set diff
  using Result = type_difference<T_Exists, T_Full>::type;
  using T_Expected = std::tuple<float, double>;

  if constexpr (std::is_same_v<Result, T_Expected>) {
    std::cout << "They are the same" << std::endl;
    return 0;
  } else {
    std::cout << "They are not the same" << std::endl;
    return 1;
  }
}
