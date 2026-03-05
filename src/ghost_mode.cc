#include "boink/simulators/vehicle/ghost_mode.h"

#include "boink/collision_group.h"
#include "boink/who_contact_callback.h"

namespace boink
{
  GhostMode::GhostMode(
      btDynamicsWorld* world, 
      RaycastVehicle* vehicle,
      const int* laps_completed)
    :world_(world),vehicle_(vehicle),laps_completed_(laps_completed)
  {
    this->reset();
  }

  void GhostMode::enable(GhostModeSettings settings)
  {
    is_sim_enabled_=true;
    is_in_ghost_mode_=false;
    settings_=std::move(settings);

    enter_timer_.reset(settings_.enter_delay);
    exit_timer_.reset(settings_.exit_delay);
    overlap_timer_.reset(settings.exit_delay_when_overlap);
    this->reset();
  }

  void GhostMode::disable()
  {
    is_sim_enabled_=false;

    this->reset();
  }

  void GhostMode::update(btScalar dt)
  {
    btScalar speed=vehicle_->getRigidBody()->getLinearVelocity().length();

    if(speed<=settings_.max_enter_speed || 
        *laps_completed_<=(int)settings_.enabled_until_completed_laps)
    {
      exit_timer_.reset();
      overlap_timer_.reset();

      if(is_in_ghost_mode_)
        return;

      enter_timer_.update(dt);
      if(enter_timer_.hasFinised())
        this->enterGhostMode();
    }

    if(speed>=settings_.min_exist_speed)
    {
      enter_timer_.reset();
      overlap_timer_.reset();

      if(!is_in_ghost_mode_)
        return;

      exit_timer_.update(dt);
      if(exit_timer_.hasFinised())
      {
        if(isOverlapping())
          overlap_timer_.reset();
        else
        {
          overlap_timer_.update(dt);
          if(overlap_timer_.hasFinised())
            this->exitGhostMode();
        }
        this->exitGhostMode();
      }
    }

    if(speed>settings_.max_enter_speed && speed<settings_.min_exist_speed)
    {
      enter_timer_.reset();
      exit_timer_.reset();
      overlap_timer_.reset();
    }
  }

  void GhostMode::enterGhostMode()
  {
    is_in_ghost_mode_=true;
    world_->removeAction(vehicle_);
    world_->removeRigidBody(vehicle_->getRigidBody());

    world_->addRigidBody(
        vehicle_->getRigidBody(),
        CollisionGroup::Vehicle,
        CollisionGroup::Static);
    world_->addAction(vehicle_);
  }

  void GhostMode::exitGhostMode()
  {
    is_in_ghost_mode_=false;
    world_->removeAction(vehicle_);
    world_->removeRigidBody(vehicle_->getRigidBody());

    world_->addRigidBody(
        vehicle_->getRigidBody(),
        CollisionGroup::Vehicle,
        CollisionGroup::Vehicle | CollisionGroup::Static);
    world_->addAction(vehicle_);
  }

  bool GhostMode::isOverlapping() const
  {
    WhoContactCallback who_callback(vehicle_->getRigidBody());
    world_->contactTest(vehicle_->getRigidBody(),who_callback);

    // TODO
    // i belive hit is also when chasiss touch ground or walls
    return who_callback.getHits().size()!=0;
  }

  void GhostMode::reset()
  {
    is_in_ghost_mode_=false;
  }
}
