#pragma once

#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include <unordered_map>

namespace boink
{
  struct WheelInfo
  {
    btVector3 position;
  };

  struct VehicleInfo
  {
    btScalar friction_slip;
    btScalar max_suspension_force;
    btScalar max_suspension_travel_cm;
    btScalar suspension_compression;
    btScalar suspension_damping;
    btScalar suspension_stiffness;

    btScalar max_steer_angle;
    btVector3 center_of_mass_cs;
    btTransform chassis_position;
    btScalar mass;
    btScalar speed;

    btScalar engine_force;
    btScalar brake;
    btScalar steering;

    int laps_completed;
    btScalar curr_lap_coverage;

    WheelInfo front_left;
    WheelInfo front_right;
    WheelInfo rear_left;
    WheelInfo rear_right;

  };

  struct TrackInfo
  {
    btVector3 position;
  };

  struct SimulationInfo
  {
    btScalar gravitational_acceleration;
    size_t vehicle_number;
    btScalar simulation_duration;

    TrackInfo track_info;
    std::unordered_map<uint64_t,VehicleInfo> vehicles_info;
  };
}
