#pragma once

#include <Eigen/Core>

namespace boink
{
  struct Kinematics
  {
    Eigen::Vector3d velocity{};
    Eigen::Vector3d acceleration{};
  };
}
