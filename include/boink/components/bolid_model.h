#pragma once

#include <Eigen/Core>

namespace boink
{
  struct BolidModel
  {
    Eigen::Vector3d rear_left_wheel;
    Eigen::Vector3d rear_right_wheel;
    Eigen::Vector3d front_left_wheel;
    Eigen::Vector3d front_right_wheel;

    Eigen::Vector3d front;
    Eigen::Vector3d direction;
  };
}
