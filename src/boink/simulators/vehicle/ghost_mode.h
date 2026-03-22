#pragma once

#include <LinearMath/btScalar.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

#include "boink/simulators/vehicle/physics/raycast_vehicle.h"
#include "boink/simulators/vehicle/ghost_mode_settings.h"
#include "boink/timer.h"
#include "boink/simulators/vehicle/lap_info.h"

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
        const LapInfo* lap_info);

    void enable(GhostModeSettings settings);
    void disable();

    void update(btScalar dt);

    bool isSimulationActive() const {return is_sim_enabled_;}
    bool isInGhostMode() const {return is_in_ghost_mode_;}
    bool isActive() const {return is_sim_enabled_;}

    const Timer& getEnterTimer() const {return enter_timer_;}
    const Timer& getExitTimer() const {return exit_timer_;}

    bool isCompletedLapsConditionMet() const 
    { return lap_info_->getCurrentLap()<(int)settings_.enabled_until_completed_laps;}
    bool isEnterSpeedConditionMet() const
    { return speed_<settings_.max_enter_speed;}
    bool isExitSpeedConditionMet() const
    { return speed_>settings_.min_exit_speed;}

    void enterGhostModeForce();
  private:
    void enterGhostMode();
    void exitGhostMode();

    void reset();
  private:
    btDynamicsWorld* world_;
    RaycastVehicle* vehicle_;
    const LapInfo* lap_info_;

    bool is_sim_enabled_=false;
    bool is_in_ghost_mode_=false;

    btScalar speed_=0.f;
    Timer enter_timer_;
    Timer exit_timer_;

    Timer force_timer_;

    GhostModeSettings settings_;
  };
}
