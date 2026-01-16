#pragma once

#include "boink/entity.h"
#include "boink/containers/unordered_container.h"
#include "boink/containers/component_view.h"
#include "boink/utils/type_difference.h"

#include <tuple>
#include <type_traits>
#include <unordered_map>

namespace boink
{
  template <typename TupleStaticComponents_,typename TupleComponents_>
  class ComponentManager;

  /**
   * @brief Manages the storage of entity components.
   */
  template<typename... StaticComponents_,typename... Components_>
  class ComponentManager<std::tuple<StaticComponents_...>,std::tuple<Components_...>>
  {
  public:
    /**
     * @brief Constructs empty object.
     */
    template <typename... Ts_>
    ComponentManager(Ts_&&... static_components) 
      :static_components_{std::forward<Ts_>(static_components)...},size_(0)
    {
      // Checks whether Ts_ types are equal to StaticComponents_ order matters.
      static_assert(
        std::is_same_v<
          std::tuple<std::remove_cvref_t<Ts_>...>,
          std::tuple<StaticComponents_...>
        >
      );
    }

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
      static_assert(
        (contains_type<SubComponents_,Components_...>::value && ...)
      );

      return ComponentView<SubComponents_...>{    
        std::tuple{
          std::span(
              getComponentContainer<SubComponents_>().data(),
              getComponentContainer<SubComponents_>().size()
          )...
        }
      };
    }

    template<typename... SubComponents_>
    auto getEntityComponents(Entity::ID entity_id)
    {
      size_t index=id_to_index_map_.at(entity_id);
      return std::tuple<SubComponents_&...>(
        getComponentContainer<SubComponents_>().at(index)...
      );
    }

    template<typename... SubComponents_>
    auto getEntityComponents(Entity::ID entity_id) const
    {
      size_t index=id_to_index_map_.at(entity_id);
      return std::tuple<const SubComponents_&...>(
        getComponentContainer<SubComponents_>().at(index)...
      );
    }

    /**
     * @brief Creates a tuple of one or more static component containers.
     *
     * @tparam StatcSubComponents_ The static component types to include in the tuple.
     *
     * @return A tuple containing StaticSubComponets_.
     */
    template<typename... StaticSubComponents_>
    std::tuple<StaticSubComponents_&...> getStaticComponentView()
    {
      return std::tuple<StaticSubComponents_&...>(
        std::get<StaticSubComponents_>(static_components_)...
      );
    }

    /**
     * @brief Adds a set of components to an entity.
     *
     * @tparam Components_ The component types being added.
     *
     * @param entity_id The unique identifier of the entity to which components are added.
     * @param components The component instances to be stored.
     */
    template <typename... Ts_>
    void addComponents(Entity::ID entity_id, Ts_&&... components)
    {
      // Checks whether Ts_ types are equal to Components_ order matters.
      static_assert(
        std::is_same_v<
          std::tuple<std::remove_cvref_t<Ts_>...>,
          std::tuple<Components_...>
        >
      );

      // If component for a given entity was added eariler do nothing.
      if(id_to_index_map_.find(entity_id)!=id_to_index_map_.end())
        return;

      (getComponentContainer<std::remove_cvref_t<Ts_>>().
       add(std::forward<Ts_>(components)),...);

      id_to_index_map_[entity_id]=size_;
      index_to_id_map_[size_]=entity_id;

      size_++;
    }

    template<typename... Ts_>
    void updateComponents(Entity::ID entity_id, Ts_&&... sub_components)
    {
      // Checks whether Ts_ are subset of types Compoents_
      static_assert(
        (contains_type<std::remove_cvref_t<Ts_>,Components_...>::value && ...)
      );

      auto it=id_to_index_map_.find(entity_id);
      // If entity doesnt exist return.
      if(it==id_to_index_map_.end())
        return;

      (getComponentContainer<std::remove_cvref_t<Ts_>>().
       update(std::forward<Ts_>(sub_components),it->second),...);
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

    /**
     * @brief Returns the entity ID corresponding to the given index.
     *
     * @param index The position of the entity in the internal container.
     *
     * @return Entity::ID The ID of the entity at the specified index.
     *
     */
    Entity::ID getIDByIndex(size_t index) const
    {
        return index_to_id_map_.at(index);
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
    static constexpr size_t STATIC_COMPONENTS_COUNT=sizeof...(StaticComponents_);
    static constexpr size_t COMPONENTS_COUNT=sizeof...(Components_);
  private:
    std::tuple<StaticComponents_...> static_components_;

    std::tuple<UnorderedContainer<Components_>...> containers_;

    std::unordered_map<Entity::ID,size_t> id_to_index_map_;
    std::unordered_map<size_t,Entity::ID> index_to_id_map_;

    size_t size_;
  };
}
