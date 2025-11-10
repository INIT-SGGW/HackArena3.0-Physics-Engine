#pragma once

#include "boink/components/component_manager.h"

#include "boink/components/transform.h"
#include "boink/components/velocity.h"

namespace boink
{
  template<typename... Components_>
  class MovementSystem
  {
  public:
    /**
      * @brief Updates all entities with Transform and Velocity components.
      *
      * @param component_manager Reference to the ComponentManager storing components.
      * @param dt Delta time.
      */
    void Update(ComponentManager<Components_...>& component_manager,
        double dt)
    {
      auto view=component_manager.template getComponentView<Transform,Velocity>();
      view.forEach(
        [=](Transform& trans, Velocity& vel)
        {
          trans.position.y()+=10+dt;
          trans.position.x()-=10+dt;

          vel.velocity.y()+=15+dt;
          vel.velocity.x()-=15+dt;
        }
      );
    }
  };
}
