#pragma once

#include <Eigen/Core>

namespace boink
{
  /**
   * @brief Represents an entity's spatial transformation.
   */
  struct Transform
  {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d rotation{Eigen::Vector3d::Zero()};
  };
}
