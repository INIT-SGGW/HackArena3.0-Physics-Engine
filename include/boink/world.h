#pragma once

#include "boink/bolid_manager.h"

#include "boink/components/transform.h"
#include "boink/components/velocity.h"

#include "boink/systems/movement_system.h"

#include "boink/utils/tuple_unpack.h"

namespace boink
{
  /**
   * @brief Represents the world simulation containing all entities and systems.
   *
   */
  class World
  {
  public:
    using BolidComponents=std::tuple<Transform,Velocity>;
    using BolidSystems=std::tuple<apply_tuple_t<MovementSystem,BolidComponents>>;
  public:
    /**
     * @brief Default initializes the world.
     *
     */
    World();

    /**
     * @brief Update the world for a simulation step.
     *
     * @param dt Delta time in seconds.
     */
    void update(double dt);
  public:
    /**
     * @brief Manages bolid entities and systems.
     */
    BolidManager<BolidComponents,BolidSystems> bolid_manager;
  };
}

