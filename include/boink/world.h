#pragma once

#if !defined(BOINK_API)
#if defined(_WIN32) || defined(__CYGWIN__)
#if defined(BOINK_BUILD_DLL)
#define BOINK_API __declspec(dllexport)
#elif defined(BOINK_USE_DLL)
#define BOINK_API __declspec(dllimport)
#else
#define BOINK_API
#endif
#else
#define BOINK_API __attribute__((visibility("default")))
#endif
#endif

#include "boink/components/transform.h"
#include "boink/components/kinematics.h"
#include "boink/components/car_inputs.h"
#include "boink/components/car_model.h"

#include "boink/car_manager.h"
#include "boink/systems/car_spawn_system.h"
#include "boink/systems/movement_system.h"
#include "boink/simulation.h"
#include "boink/debug_render.h"

namespace boink
{
  /**
   * @brief Represents the world simulation containing all entities and systems.
   *
   */
  class BOINK_API World
  {
  public:
    using CarComponents=std::tuple<CarInput,Transform,Kinematics>;
    using CarSystems=std::tuple<CarSpawnSystem,MovementSystem>;
  public:
    World(const CarModel& car_model);

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
    void setDebuger(DebugRender* dbg);
    void addGround(const btVector3& dims, const btVector3& pos);
    void addSphere(btScalar radius, const btVector3& pos);
  public:
    /**
     * @brief Manages car entities and systems.
     */
    CarManager<CarComponents,CarSystems> car_manager;
  private:
    Simulation simulation;
    double time_passed_=0.0;
  };
}

