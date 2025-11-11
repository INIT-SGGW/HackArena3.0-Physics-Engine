#pragma once

#include "boink/entity.h"
#include "boink/containers/unordered_container.h"
#include "boink/containers/component_view.h"

#include <tuple>
#include <unordered_map>

namespace boink
{

  /**
   * @brief Manages the storage of entity components.
   */
  template<typename... Components_>
  class ComponentManager
  {
  public:
    /**
     * @brief Constructs empty object.
     */
    ComponentManager()
      :size_(0)
    {}

    /**
     * @brief Creates a view into one or more component containers.
     *
     * This function constructs a @ref ComponentView that provides non-owning, 
     * contiguous access to the selected component types.
     * 
     * @tparam SubComponents_ The component types to include in the view.
     *
     * @return A @ref ComponentView containing spans to the selected component arrays.
     *
     * @warning Any operation that causes a reallocation in a component container 
     * (such as adding new entities beyond its current capacity) will invalidate 
     * the spans held by the returned view.
     *
     * @see ComponentView
     */
    template<typename... SubComponents_>
    ComponentView<SubComponents_...> getComponentView()
    {
      return ComponentView<SubComponents_...>{    
        std::tuple{
          std::span(
              getComponentContainer<SubComponents_>().data(),
              getComponentContainer<SubComponents_>().size()
          )...
        }
      };
    }

    /**
     * @brief Adds a set of components to an entity.
     *
     * @tparam Components_ The component types being added.
     *
     * @param entity_id The unique identifier of the entity to which components are added.
     * @param components The component instances to be stored.
     */
    void addComponents(Entity::ID entity_id,Components_... components)
    {
      // If component for a given entity was added eariler do nothing.
      if(id_to_index_map_.find(entity_id)!=id_to_index_map_.end())
        return;
      
      (getComponentContainer<Components_>().add(components),...);
      id_to_index_map_[entity_id]=size_;

      size_++;
    }

    /**
     * @brief Removes all components associated with a specific entity ID.
     *
     * This function removes the entity's components from the storage. It implements
     * a **swap-and-pop** technique to maintain tight packing of component arrays:
     * the component data at the position of the deleted element is replaced by the
     * component data from the last active element, and then the size is reduced.
     * This invalidates the indices of the moved component, which is updated in the
     * internal mapping.
     *
     * @param entity_id The unique identifier of the entity.
     *
     * @return @c true if the components were found and successfully removed, 
     * @c false otherwise.
     */
    bool removeComponents(Entity::ID entity_id)
    {
      Entity::ID del_id=entity_id;
      auto it_del=id_to_index_map_.find(del_id);
      if(it_del==id_to_index_map_.end())
        return false;

      size_t i_del=it_del->second;
      size_t i_last=size_-1;
      Entity::ID last_id=index_to_id_map_[i_last];

      // Removes element at i_del and moves data from end to i_del
      (getComponentContainer<Components_>().remove(i_del),...);

      id_to_index_map_.erase(del_id);
      index_to_id_map_.erase(i_last);

      id_to_index_map_[last_id]=i_del;
      index_to_id_map_[i_del]=last_id;
      
      size_--;

      return true;
    }

    Entity::ID getIDbyIndex(size_t index)
    {
        return index_to_id_map_[index];
    }

  private:
    /**
     * @brief Gets the vector of components of type Component_.
     *
     * @tparam Component_ An component type.
     *
     * @return A reference to a ComponentContainer.
     */
    template<typename Component_>
    UnorderedContainer<Component_>& getComponentContainer()
    {
      return std::get<UnorderedContainer<Component_>>(containers_);
    }

    /**
     * @brief Gets the ComponentContainer of type Component_.
     *
     * @tparam Component_ An component type.
     *
     * @return A const reference to a ComponentContainer.
     */
    template<typename Component_>
    const UnorderedContainer<Component_>& getComponentContainer() const
    {
      return std::get<UnorderedContainer<Component_>>(containers_);
    }
  public:
    static constexpr size_t COMPONENTS_COUNT=sizeof...(Components_);
  private:
    std::tuple<UnorderedContainer<Components_>...> containers_;
    std::unordered_map<Entity::ID,size_t> id_to_index_map_;
    std::unordered_map<size_t,Entity::ID> index_to_id_map_;

    size_t size_;
  };
}
