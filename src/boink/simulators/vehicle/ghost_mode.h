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
    bool isOverlapping() const {return is_overlapping_;}

    const Timer& getEnterTimer() const {return enter_timer_;}
    const Timer& getExitTimer() const {return exit_timer_;}
    const Timer& getOverlapTimer() const {return overlap_timer_;}

    bool isCompletedLapsConditionMet() const 
    { return *laps_completed_<(int)settings_.enabled_until_completed_laps;}
    bool isEnterSpeedConditionMet() const
    { return speed_<settings_.max_enter_speed;}
    bool isExitSpeedConditionMet() const
    { return speed_>settings_.min_exit_speed;}
  private:
    void enterGhostMode();
    void exitGhostMode();

    void doHitTest();
  private:
    btDynamicsWorld* world_;
    RaycastVehicle* vehicle_;
    const int* laps_completed_;

    bool is_sim_enabled_=false;
    bool is_overlapping_=false;
    btScalar speed_=0.f;

    bool is_in_ghost_mode_=false;

    Timer enter_timer_;
    Timer exit_timer_;
    Timer overlap_timer_;

    GhostModeSettings settings_;
  };
}
