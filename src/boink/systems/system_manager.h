#pragma once

#include "boink/components/component_manager.h"
#include "boink/simulation.h"

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
    template<typename Chuj1, typename Chuj2>
    SystemManager(ComponentManager<Chuj1,Chuj2>& component_manager)
    {
      
    }
    /**
     * @brief Setup all systems. Is called only once.
     *
     * @param component_manager Reference to the ComponentManager storing components.
     * @param dt Delta time.
     */
    template <typename TupleStaticComponents_, typename TupleComponents_>
    void Setup(
        ComponentManager<TupleStaticComponents_,TupleComponents_>& component_manager,
        Simulation& sim,
        double dt)
    {
      std::apply([&,dt](auto&... sys) {
        (([&,dt] {
          if constexpr (requires { sys.Setup(component_manager,sim, dt); }) {
            sys.Setup(component_manager,sim, dt);
          }
        }()), ...);
      }, systems_);
    }

    /**
     * @brief Updates all systems.
     *
     * @param component_manager Reference to the ComponentManager storing components.
     * @param dt Delta time.
     */
    template <typename TupleStaticComponents_, typename TupleComponents_>
    void Update(
        ComponentManager<TupleStaticComponents_,TupleComponents_>& component_manager,
        Simulation& sim,
        double dt)
    {
      std::apply([&,dt](auto&... sys) {
        (([&,dt] {
          if constexpr (requires { sys.Update(component_manager,sim, dt); }) {
            sys.Update(component_manager,sim, dt);
          }
        }()), ...);
      }, systems_);
    }
  private:
    std::tuple<Systems_...> systems_;
  };
}
