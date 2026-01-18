#pragma once

#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include <vector>

namespace boink
{
  struct WheelInfo
  {
    btVector3 position;
  };

  struct VehicleInfo
  {
    btScalar max_steer_angle;
    btVector3 center_of_mass_cs;
    btVector3 chassis_position;
    btScalar mass;
    btScalar speed;

    btScalar engine_force;
    btScalar brake;
    btScalar steering;

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
    std::vector<VehicleInfo> vehicles_info;
  };
}
