#pragma once


#include "boink/components/transform.h"
#include "boink/components/kinematics.h"
#include "boink/components/car_inputs.h"
#include "boink/components/car_model.h"

#include "boink/car_manager.h"
#include "boink/systems/car_spawn_system.h"
#include "boink/systems/movement_system.h"
#include "boink/systems/debug_system.h"

namespace boink
{
  /**
   * @brief Represents the world simulation containing all entities and systems.
   *
   */
  class World
  {
  public:
    using CarComponents=std::tuple<CarInput,Transform,Kinematics>;
    using CarSystems=std::tuple<CarSpawnSystem,MovementSystem>;
  public:
    World(const CarModel& car_model);
    World(CarModel&& car_model);

    /**
     * @brief Start simulation.
     *
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
     * @brief Manages car entities and systems.
     */
    CarManager<CarComponents,CarSystems> car_manager;
  private:
    double time_passed_;
  };
}

