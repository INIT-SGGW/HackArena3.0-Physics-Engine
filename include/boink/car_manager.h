#pragma once

#include <cassert>
#include <tuple>
#include <type_traits>
#include <unordered_set>

#include "boink/components/car_model.h"
#include "boink/components/component_manager.h"
#include "boink/entity.h"
#include "boink/systems/system_manager.h"
#include "boink/utils/type_difference.h"

namespace boink {

template <typename TupleComponents_, typename TupleSystems_>
class CarManager;

/**
 * @brief Manages car entities composed of components and systems.
 *
 * Handles entity creation, destruction, and component management.
 * Systems are automatically updated each frame through SystemManager.
 */
template <typename... Components_, typename... Systems_>
class CarManager<std::tuple<Components_...>, std::tuple<Systems_...>> {
 public:
  using ID = Entity::ID;

 public:
  CarManager(const CarModel& car_model) : component_manager_(car_model) {}

  CarManager(CarModel&& car_model) : component_manager_(std::move(car_model)) {}

  ~CarManager() {}

  /**
   * @brief Sets up all systmes on car entities
   *
   * @param dt Delta time.
   */
  void SetupSystems(double dt) {
    system_manager_.Setup(component_manager_, dt);
  }

  /**
   * @brief Updates all systmes on car entities
   *
   * @param dt Delta time.
   */
  void UpdateSystems(double dt) {
    system_manager_.Update(component_manager_, dt);
  }

  /**
   * @brief Create a new car entity.
   *
   * @param components Component instances to create car with.
   *
   * @return Unique car identifier.
   */
  template <typename... Ts_>
  ID addCar(Ts_&&... components) {
    // Checks whether Ts_ types are equal to Components_ order matters.
    static_assert(std::is_same_v<std::tuple<std::remove_cvref_t<Ts_>...>,
                                 std::tuple<Components_...>>);

    ID new_id;
    if (avail_ids_.empty())
      new_id = cars_.size();
    else {
      new_id = *avail_ids_.begin();
      avail_ids_.erase(avail_ids_.begin());
    }

    auto pair = cars_.insert(new_id);
    assert(pair.second);

    component_manager_.addComponents(new_id, std::forward<Ts_>(components)...);
    return new_id;
  }

  /**
   * @brief Creates a new car entity with default component instances.
   *
   * @return Unique car indetifier.
   */
  ID addCar() { return addCar(Components_{}...); }

  template <typename... Ts_>
  void updateCar(ID car_id, Ts_&&... sub_components) {
    // Checks whether Ts_ are subset of types Compoents_
    static_assert((
        contains_type<std::remove_cvref_t<Ts_>, Components_...>::value && ...));

    if (!cars_.contains(car_id)) return;

    component_manager_.updateComponents(car_id,
                                        std::forward<Ts_>(sub_components)...);
  }

  template <typename... SubComponents_>
  auto getCarComponents(ID car_id) {
    return component_manager_.template getEntityComponents<SubComponents_...>(
        car_id);
  }

  template <typename... SubComponents_>
  auto getCarComponents(ID car_id) const {
    return component_manager_.template getEntityComponents<SubComponents_...>(
        car_id);
  }

  template <typename... StaticSubComponents_>
  std::tuple<StaticSubComponents_&...> getCarStaticComponents() {
    return component_manager_
        .template getStaticComponentView<StaticSubComponents_...>();
  }

  /**
   * @brief Removes car.
   *
   * @param id Id of the car to delete.
   *
   * @return True if car removed, false otherwise.
   */
  bool removeCar(ID id) {
    if (!cars_.contains(id)) return false;

    cars_.erase(id);
    avail_ids_.insert(id);

    bool ret = component_manager_.removeComponents(id);

    assert(ret);
    return ret;
  }

 private:
  std::unordered_set<ID> cars_;
  std::unordered_set<ID> avail_ids_;

  ComponentManager<std::tuple<CarModel>, std::tuple<Components_...>>
      component_manager_;
  SystemManager<Systems_...> system_manager_;
};
}  // namespace boink
