#pragma once

#include "boink/components/component_manager.h"

#include <tuple>

namespace boink
{

  /**
   * @brief Manages a set of systems and updates them.
   */
  template<typename... Systems_>
  class SystemManager
  {
  public:
    /**
     * @brief Setup all systems. Is called only once.
     *
     * @param component_manager Reference to the ComponentManager storing components.
     * @param dt Delta time.
     */
    template <typename... Components_>
    void Setup(ComponentManager<Components_...>& component_manager,double dt)
    {
      ((
        [&,dt](){
          auto& sys=std::get<Systems_>(systems_);
          if constexpr(requires{sys.Setup(component_manager,dt);})
            sys.Setup(component_manager, dt);
        }()),
        ...
      );
    }

    /**
     * @brief Updates all systems.
     *
     * @param component_manager Reference to the ComponentManager storing components.
     * @param dt Delta time.
     */
    template <typename... Components_>
    void Update(ComponentManager<Components_...>& component_manager,double dt)
    {
      ((
        [&,dt]{
          auto& sys=std::get<Systems_>(systems_);
          if constexpr(requires{sys.Update(component_manager,dt);})
            sys.Update(component_manager, dt);
        }()),
        ...
      );
    }
  private:
    std::tuple<Systems_...> systems_;
  };
}
