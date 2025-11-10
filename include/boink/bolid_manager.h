#pragma once

#include "boink/entity.h"

#include "boink/components/component_manager.h"
#include "boink/systems/system_manager.h"

#include <tuple>
#include <unordered_set>
#include <cassert>

namespace boink
{

  template<typename TupleComponents_,typename TupleSystems_>
  class BolidManager;

  /**
   * @brief Manages bolid entities composed of components and systems.
   *
   * Handles entity creation, destruction, and component management.
   * Systems are automatically updated each frame through SystemManager.
   */
  template<typename... Components_,typename... Systems_>
  class BolidManager<std::tuple<Components_...>,std::tuple<Systems_...>>
  {
  public:
    using ID=Entity::ID;
  public:
    /**
     * @brief Updates all systmes on bolid entities
     *
     * @param dt Delta time.
     */
    void UpdateSystems(double dt)
    {
      system_manager_.Update(component_manager_,dt);
    }

    /**
     * @brief Create a new bolid entity.
     *
     * @param components Component instances to create bolid with.
     *
     * @return Unique bolid identifier.
     */
    ID addBoild(Components_... components)
    {
      ID new_id;
      if(avail_ids_.empty())
        new_id=bolids_.size();
      else
      {
        new_id=*avail_ids_.begin();
        avail_ids_.erase(avail_ids_.begin());
      }

      auto pair=bolids_.insert(new_id);
      assert(pair.second);

      component_manager_.addComponents(new_id,components...);
      return new_id;
    }

    /**
     * @brief Creates a new bolid entity with default component instances.
     *
     * @return Unique bolid indetifier.
     */
    ID addBoild()
    {
      return addBoild(Components_{}...);
    }

    /**
     * @brief Removes bolid.
     *
     * @param id Id of the bolid to delete.
     *
     * @return True if bolid removed, false otherwise.
     */
    bool removeBolid(ID id)
    {
      if(!bolids_.contains(id))
        return false;

      bolids_.erase(id);
      avail_ids_.insert(id);

      bool ret=component_manager_.removeComponents(id);
      
      assert(ret);
      return ret;
    }
  private:
    std::unordered_set<ID> bolids_;
    std::unordered_set<ID> avail_ids_;

    ComponentManager<Components_...> component_manager_;
    SystemManager<std::tuple<Components_...>,std::tuple<Systems_...>> system_manager_;
  };
}
