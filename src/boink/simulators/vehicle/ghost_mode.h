#pragma once

#include <LinearMath/btScalar.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

#include "boink/simulators/vehicle/physics/raycast_vehicle.h"
#include "boink/simulators/vehicle/ghost_mode_settings.h"
#include "boink/timer.h"

namespace boink
{
  class GhostGui;
  class GhostMode
  {
  public:
    friend class GhostGui;
  public:
    GhostMode(
        btDynamicsWorld* world, 
        RaycastVehicle* vehicle,
        const int* laps_completed);

    void enable(GhostModeSettings settings);
    void disable();

    void update(btScalar dt);

    bool isInGhostMode() const {return is_in_ghost_mode_;}
  private:
    void enterGhostMode();
    void exitGhostMode();

    bool isOverlapping() const;
  private:
    btDynamicsWorld* world_;
    RaycastVehicle* vehicle_;
    const int* laps_completed_;

    bool is_sim_enabled_=false;

    bool is_in_ghost_mode_=false;

    Timer enter_timer_;
    Timer exit_timer_;
    Timer overlap_timer_;

    GhostModeSettings settings_;
  };
}
