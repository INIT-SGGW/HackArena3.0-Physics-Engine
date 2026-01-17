#include "boink/simulation.h"

#include <LinearMath/btIDebugDraw.h>
#include <piksel/model.hh>

#include "boink/exception.h"
#include "boink/simulation/track.h"

#include <memory>
#include <algorithm>
#include <cassert>

namespace boink
{
  Simulation::Simulation(
      std::string_view track_filepath)
    :collision_configuration_(new btDefaultCollisionConfiguration()),
    dispatcher_(new btCollisionDispatcher(collision_configuration_.get())),
    overlapping_pair_cache_(new btDbvtBroadphase()),
    solver_(new btSequentialImpulseConstraintSolver()),
    dynamics_world_(new btDiscreteDynamicsWorld(
          dispatcher_.get(),overlapping_pair_cache_.get(),
          solver_.get(),collision_configuration_.get())),
    track_(track_filepath,dynamics_world_)
  {
    dynamics_world_->setGravity(btVector3(0, -kGravitationalAcceleration, 0));
  } 

  void Simulation::registerDebugDrawer(btIDebugDraw* dbg)
  {
    dynamics_world_->setDebugDrawer(dbg);
    dbg->setDebugMode(
        btIDebugDraw::DBG_DrawWireframe |
        btIDebugDraw::DBG_DrawConstraints |
        btIDebugDraw::DBG_DrawContactPoints |
        btIDebugDraw::DBG_DrawAabb
    );
  }

  void Simulation::step(btScalar dt) noexcept
  {
    // With large dt simulation behaves strangely.
    // Must use hard clamp or assert
#ifndef RASPBERRY_PI
    assert(dt<kMaxDeltaTime);
#endif

    dt=std::min(dt,kMaxDeltaTime);

    int steps=dynamics_world_->stepSimulation(dt, kMaxSubSteps,kFixedDeltaTime);
    simulation_duration_+=steps*kFixedDeltaTime;

    dynamics_world_->debugDrawWorld();
  }

  Simulation::ObjectID Simulation::addVehicle(const Vehicle::CreationInfo& info)
  {
    vehicles_.emplace_back(info,dynamics_world_);
    return vehicles_.size()-1;
  }

  void Simulation::removeVehicle(ObjectID id)
  {
    if(id >=vehicles_.size())
      throw Exception(
          Exception::Type::NotFoundError,
          "Vehicle with a given ID does not exist");
    vehicles_.erase(vehicles_.cbegin()+id);
  }

  Vehicle& Simulation::getVehicle(Simulation::ObjectID id)
  {
    if(id >=vehicles_.size())
      throw Exception(
          Exception::Type::NotFoundError,
          "Vehicle with a given ID does not exist");
    return vehicles_[id];
  }

  Track& Simulation::getTrack()
  {
    return track_;
  }
}   
    
    
