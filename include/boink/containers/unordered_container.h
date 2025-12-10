#pragma once

#include <cassert>
#include <type_traits>
#include <vector>

namespace boink {
/**
 * @brief A generic, unordered container that provides O(1) removal.
 *
 * @tparam T The type to be stored.
 *
 * @note The order of components is **not preserved** after removal.
 */
template <typename T>
class UnorderedContainer {
 public:
  /**
   * @brief Adds an element to the container by an lvalue.
   *
   * @param value The element to be added.
   */
  template <typename U>
  void add(U&& value) {
    values_.push_back(std::forward<U>(value));
  }

  /**
   * @brief Set value at given index.
   *
   * @param value Value to assign.
   * @param index Target index.
   */
  template <typename U>
  void update(U&& value, size_t index) {
    static_assert(std::is_same_v<T, std::remove_cvref_t<U>>);
    values_[index] = std::forward<U>(value);
  }

  /**
   * @brief Removes the element at the specified index.
   *
   * @param index The index of the element to remove.
   */
  void remove(size_t index) {
    assert(index < values_.size());

    values_[index] = std::move(values_.back());
    values_.pop_back();
  }

  T& at(size_t index) { return values_.at(index); }

  const T& at(size_t index) const { return values_.at(index); }
  /**
   * @brief Returns a pointer to the underlying data.
   *
   * @return Pointer to the first element in the container, or nullptr if empty.
   */
  T* data() noexcept { return values_.data(); }

  /**
   * @brief Returns a const pointer to the underlying data.
   *
   * @return Const pointer to the first element in the container, or nullptr if
   * empty.
   */
  const T* data() const noexcept { return values_.data(); }

  /**
   * @brief Returns the number of elements in the container.
   *
   * @return size_t The current size of the container.
   */
  std::size_t size() const noexcept { return values_.size(); }

 private:
  std::vector<T> values_;
};
}  // namespace boink
