#pragma once

#include <Eigen/Core>

namespace boink
{
  /**
   * @brief Represents model of a bolid it is constant during simulation.
   */
  struct BolidModel
  {
    // TODO
    // Make it vars const and create ctor
    Eigen::Vector3d rear_left_wheel;
    Eigen::Vector3d rear_right_wheel;
    Eigen::Vector3d front_left_wheel;
    Eigen::Vector3d front_right_wheel;
    double max_steer_angle_deg;

    Eigen::Vector3d front;
    Eigen::Vector3d direction; // Normalized vector
    Eigen::Vector3d normal; // Normalized. Points to the top of a car 
                            // perpedicular to wheels plane
  };
}
