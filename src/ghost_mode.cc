#include "boink/simulators/vehicle/ghost_mode.h"

#include <LinearMath/btScalar.h>

#include "boink/bullet_user_data.h"
#include "boink/collision_group.h"
#include "boink/who_contact_callback.h"
#include "boink/exception.h"

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
      throw Exception(
          Exception::Type::InvalidArgumentError,
          "Min exit speed cannot be lower than max enter speed");

    is_sim_enabled_=true;
    enterGhostMode();
    settings_=std::move(settings);

    this->reset();
  }

  void GhostMode::disable()
  {
    is_sim_enabled_=false;
    this->reset();

    if(isInGhostMode())
      this->exitGhostMode();
  }

  void GhostMode::update(btScalar dt)
  {
    this->doHitTest();

    if(!is_sim_enabled_)
    {
      if(isInGhostMode())
      {
        force_timer_.update(dt);
        if(force_timer_.hasFinised()&&!isOverlapping())
          this->exitGhostMode();
      }
      return;
    }

    if(isOverlapping())
      overlap_timer_.reset();
    else
      overlap_timer_.update(dt);

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

      if(exit_timer_.hasFinised()&&overlap_timer_.hasFinised())
        this->exitGhostMode();
    }

    if(!this->isEnterSpeedConditionMet() && !this->isExitSpeedConditionMet())
    {
      enter_timer_.reset();
      exit_timer_.reset();
    }
  }

  void GhostMode::enterGhostModeForce()
  {
    this->enterGhostMode();

    exit_timer_.reset();
    force_timer_.reset();
  }

  void GhostMode::enterGhostMode()
  {
    is_in_ghost_mode_=true;
    enter_timer_.reset();

    world_->removeAction(vehicle_);
    world_->removeRigidBody(vehicle_->getRigidBody());

    world_->addRigidBody(
        vehicle_->getRigidBody(),
        Collision::Group::Vehicle,
        Collision::Group::Static);
    world_->addAction(vehicle_);
  }

  void GhostMode::exitGhostMode()
  {
    is_in_ghost_mode_=false;
    exit_timer_.reset();

    world_->removeAction(vehicle_);
    world_->removeRigidBody(vehicle_->getRigidBody());

    world_->addRigidBody(
        vehicle_->getRigidBody(),
        Collision::Group::Vehicle,
        Collision::Group::Vehicle | Collision::Group::Static);
    world_->addAction(vehicle_);
  }

 void GhostMode::doHitTest()
  {
    WhoContactCallback who_callback(vehicle_->getRigidBody());
    // TODO
    // i belive hit is also when chasiss touch ground or walls
    world_->contactTest(vehicle_->getRigidBody(),who_callback);
    const auto& hits=who_callback.getHits();

    for(size_t i=0;i<hits.size();i++)
    {
      if(hits[i]->getUserPointer())
      {
        BulletUserData* bullet_user_data=
          reinterpret_cast<BulletUserData*>(hits[i]->getUserPointer());
        
        if(bullet_user_data->getType()==BulletUserData::Type::Vehicle)
        {
          is_overlapping_=true;
          return;
        }
      }
#ifndef NDEBUG
      // TODO put it into logger
      else
        throw Exception(
            Exception::Type::InternalError,
            "Collision object does not have set BulletUserDataPointer");
#endif
    }

    is_overlapping_=false;
  }

 void GhostMode::reset()
 {
    enter_timer_.reset(settings_.enter_delay);
    exit_timer_.reset(settings_.exit_delay);
    overlap_timer_.reset(settings_.exit_delay_when_overlap);
    overlap_timer_.setElapsedToFinish();

    force_timer_.reset(settings_.exit_delay);
 }
}
