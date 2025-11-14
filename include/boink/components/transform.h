#pragma once

#include <Eigen/Core>

namespace boink
{
  /**
   * @brief Represents an entity's spatial transformation.
   */
  struct Transform
  {
    Eigen::Vector3d position{};
    Eigen::Matrix3d rotation{};
  };
}
