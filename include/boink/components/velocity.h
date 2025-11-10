#pragma once

#include <Eigen/Core>

namespace boink
{
  /**
   * @brief Represents an entity's velocity.
   */
  struct Velocity
  {
    Eigen::Vector3d velocity=Eigen::Vector3d::Zero();
  };
}
