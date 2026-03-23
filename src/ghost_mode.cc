#include "boink/simulators/vehicle/ghost_mode.h"

#include <LinearMath/btScalar.h>

#include "boink/logger.h"

namespace boink
{
  GhostMode::GhostMode(
      btDynamicsWorld* world, 
      RaycastVehicle* vehicle,
      const LapInfo* lap_info)
    :world_(world),vehicle_(vehicle),lap_info_(lap_info)
  {
  }

  void GhostMode::enable(GhostModeSettings settings)
  {
    if(settings.min_exit_speed<settings.max_enter_speed)
       BOINK_ERROR("Min exit speed cannot be lower than max enter speed");

    is_sim_enabled_=true;
    settings_=std::move(settings);
    enterGhostMode();

    this->reset();
  }

  void GhostMode::disable()
  {
    is_sim_enabled_=false;
    this->reset();
  }

  void GhostMode::update(btScalar dt)
  {
    if(!isSimulationActive())
    {
      if(isInGhostMode())
      {
        if(force_timer_.hasFinised())
          this->exitGhostMode();
        force_timer_.update(dt);
      }

      return;
    }

    speed_=vehicle_->getRigidBody()->getLinearVelocity().length();

    if(this->isEnterSpeedConditionMet()|| 
       isCompletedLapsConditionMet()) 
    {
      exit_timer_.reset();

      if(isInGhostMode())
        return;

      enter_timer_.update(dt);
      if(enter_timer_.hasFinised())
        this->enterGhostMode();
    }

    if(this->isExitSpeedConditionMet())
    {
      enter_timer_.reset();

      if(!isInGhostMode())
        return;

      exit_timer_.update(dt);

      if(exit_timer_.hasFinised())
        this->exitGhostMode();
    }

    if(!this->isEnterSpeedConditionMet() && !this->isExitSpeedConditionMet())
      this->reset();
  }

  void GhostMode::enterGhostModeForce()
  {
    this->enterGhostMode();
  }

  void GhostMode::enterGhostMode()
  {
    is_in_ghost_mode_=true;
    this->reset();

    world_->getPairCache()->cleanProxyFromPairs(
        vehicle_->getRigidBody()->getBroadphaseHandle(), 
        world_->getDispatcher()
    );

    vehicle_->getRigidBody()->activate(true);
  }

  void GhostMode::exitGhostMode()
  {
    is_in_ghost_mode_=false;
    this->reset();

    world_->getPairCache()->cleanProxyFromPairs(
        vehicle_->getRigidBody()->getBroadphaseHandle(), 
        world_->getDispatcher()
    );

    vehicle_->getRigidBody()->activate(true);
  }

 void GhostMode::reset()
 {
    enter_timer_.reset(settings_.enter_delay);
    exit_timer_.reset(settings_.exit_delay);

    force_timer_.reset(settings_.exit_delay_when_overlap);
 }
}
