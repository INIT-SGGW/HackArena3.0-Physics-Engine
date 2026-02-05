#include "boink/simulation.h"

#include <LinearMath/btIDebugDraw.h>
#include <piksel/model.hh>

#include "boink/exception.h"
#include "boink/simulation/track.h"
#include "boink/simulation/simulation_info.h"

#include <memory>
#include <algorithm>
#include <cassert>
#include <sstream>
#include <unordered_map>

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

  void Simulation::registerDebugDrawer(DebugDrawer* dbg)
  {
    p_debug_drawer=dbg;
    dynamics_world_->setDebugDrawer(p_debug_drawer);
  }

  void Simulation::step(btScalar dt) noexcept
  {
    if(p_debug_drawer && p_debug_drawer->isSimulationToFreeze())
      return;
    updateDebugInfo();

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
    ObjectID id=available_ids++;
    vehicles_.insert({id,Vehicle(info,dynamics_world_)});
    return id;
  }

  void Simulation::removeVehicle(ObjectID id)
  {
    auto it=vehicles_.find(id);
    if(it==vehicles_.end())
    {
      std::stringstream ss;
      ss<<"Vehicle with ID="<<id<<" does not exist";
      throw Exception(
          Exception::Type::NotFoundError,
          ss.str());
    }
    vehicles_.erase(it);
  }

  Vehicle& Simulation::getVehicle(Simulation::ObjectID id)
  {
    auto it=vehicles_.find(id);
    if(it==vehicles_.end())
    {
      std::stringstream ss;
      ss<<"Vehicle with ID="<<id<<" does not exist";
      throw Exception(
          Exception::Type::NotFoundError,
          ss.str());
    }
    return it->second;
  }

  Track& Simulation::getTrack()
  {
    return track_;
  }

  void Simulation::updateDebugInfo()
  {
    if(!p_debug_drawer)
      return;

    this->readDebugInfo();
    this->writeDebugInfo();
  }

  void Simulation::readDebugInfo()
  {
    if(!p_debug_drawer)
      return;

    SimulationInfo& sim_info=p_debug_drawer->getSimulationInfo();

    for(auto it=sim_info.vehicles_info.begin();
        it!=sim_info.vehicles_info.end();)
    {
      const auto& key=it->first;
      
      if(auto local_vehicle_it=vehicles_.find(key);
          local_vehicle_it!=vehicles_.end())
      {
        const auto& vehicle_info=it->second;
        btRaycastVehicle::btVehicleTuning tuning;
        tuning.m_frictionSlip=vehicle_info.friction_slip;
        tuning.m_maxSuspensionForce=vehicle_info.max_suspension_force;
        tuning.m_maxSuspensionTravelCm=vehicle_info.max_suspension_travel_cm;
        tuning.m_suspensionCompression=vehicle_info.suspension_compression;
        tuning.m_suspensionDamping=vehicle_info.suspension_damping;
        tuning.m_suspensionStiffness=vehicle_info.suspension_stiffness;
        
        local_vehicle_it->second.setTuning(tuning);

        it++;
      }
      else
      {
        it=sim_info.vehicles_info.erase(it);
      }
    }
  }

  void Simulation::writeDebugInfo()
  {
    SimulationInfo& info=p_debug_drawer->getSimulationInfo();
    info.gravitational_acceleration=kGravitationalAcceleration;
    info.simulation_duration=this->getSimulationDuration();
    info.vehicle_number=this->getVehicleNumber();

    TrackInfo track_info;
    track_info.position=this->getTrack().getPosition();
    info.track_info=std::move(track_info);

    std::unordered_map<uint64_t,VehicleInfo> vehicles_info;
    vehicles_info.reserve(this->getVehicleNumber());
    for(const auto& [key,vehicle]:vehicles_)
    {
      VehicleInfo vehicle_info;
      vehicle_info.brake=0;
      vehicle_info.center_of_mass_cs=vehicle.getCenterOfMassCS();
      vehicle_info.chassis_position=vehicle.getChassisWorldTransform().getOrigin();
      vehicle_info.engine_force=0;
      vehicle_info.mass=vehicle.getMass();
      vehicle_info.max_steer_angle=0;
      vehicle_info.speed=vehicle.getSpeed();
      vehicle_info.steering=0;

      const auto& tuning=vehicle.getTuning();
      vehicle_info.friction_slip=tuning.m_frictionSlip;
      vehicle_info.max_suspension_force=tuning.m_maxSuspensionForce;
      vehicle_info.max_suspension_travel_cm=tuning.m_maxSuspensionTravelCm;
      vehicle_info.suspension_compression=tuning.m_suspensionCompression;
      vehicle_info.suspension_damping=tuning.m_suspensionDamping;
      vehicle_info.suspension_stiffness=tuning.m_suspensionStiffness;

      vehicle_info.front_left.position=
        vehicle.getWheelWorldTransform(WheelPosition::FrontLeft).getOrigin();
      vehicle_info.front_right.position=
        vehicle.getWheelWorldTransform(WheelPosition::FrontRight).getOrigin();
      vehicle_info.rear_left.position=
        vehicle.getWheelWorldTransform(WheelPosition::RearLeft).getOrigin();
      vehicle_info.rear_right.position=
        vehicle.getWheelWorldTransform(WheelPosition::RearRight).getOrigin();

      vehicles_info[key]=std::move(vehicle_info);
    }
    info.vehicles_info=std::move(vehicles_info);
  }
}   
    
