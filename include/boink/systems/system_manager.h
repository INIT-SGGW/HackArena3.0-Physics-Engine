#pragma once

#include "boink/components/component_manager.h"

#include <tuple>

namespace boink
{
  template<typename TupleComponents_,typename TupleSystems_>
  class SystemManager;

  /**
   * @brief Manages a set of systems and updates them.
   */
  template<typename... Components_,typename... Systems_>
  class SystemManager<std::tuple<Components_...>,std::tuple<Systems_...>>
  {
  public:
    /**
     * @brief Updates all systems.
     *
     * @param component_manager Reference to the ComponentManager storing components.
     * @param dt Delta time.
     */
    void Update(ComponentManager<Components_...>& component_manager,double dt)
    {
      (std::get<Systems_>(systems_).Update(component_manager,dt),...);
    }
  private:
    std::tuple<Systems_...> systems_;
  };
}
