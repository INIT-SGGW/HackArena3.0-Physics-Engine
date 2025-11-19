#pragma once

#include <span>
#include <tuple>

namespace boink {
/**
 * @brief Provides a non-owning view over multiple component arrays.
 *
 * @tparam Components_ Component types included in view.
 */
template <typename... Components_>
struct ComponentView {
 public:
  /**
   * @brief Iterate over all entities in the view, applying a function.
   *
   * @tparam Func_ Callable type.
   * @param func Function to apply to each entity's components.
   */
  template <typename Func_>
  void forEach(Func_&& func) {
    size_t size = std::get<0>(spans_).size();
    for (size_t i = 0; i < size; i++)
      func(std::get<std::span<Components_>>(spans_)[i]...);
  }

 public:
  std::tuple<std::span<Components_>...> spans_;
};
};  // namespace boink
