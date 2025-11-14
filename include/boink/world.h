#pragma once

#include "boink/bolid_manager.h"

#include "boink/components/transform.h"
#include "boink/components/kinematics.h"
#include "boink/components/bolid_model.h"
#include "boink/components/bolid_inputs.h"

#include "boink/systems/debug_system.h"
#include "boink/systems/bolid_spawn_system.h"
#include "boink/systems/movement_system.h"

namespace boink
{
  /**
   * @brief Represents the world simulation containing all entities and systems.
   *
   */
  class World
  {
  public:
    using BolidComponents=std::tuple<BolidModel,BolidInput,Transform,Kinematics>;
    using BolidSystems=std::tuple<BolidSpawnSystem,MovementSystem,DebugSystem>;
  public:
    /**
     * @brief Default initializes the world.
     *
     */
    World();

    /**
     * @brief Start simulation.
     *
     * @param dt Delta time in seconds.
     */
    void start(double dt);

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

