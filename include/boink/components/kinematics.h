#pragma once

#include <Eigen/Core>

namespace boink
{
  /**
   * @brief Describes body motion.
   */
  struct Kinematics
  {
    Eigen::Vector3d velocity{};
    double acceleration{};
  };
}
