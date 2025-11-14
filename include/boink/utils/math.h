#pragma once

#include <Eigen/Core>

namespace boink
{
  namespace math
  {
    Eigen::Matrix3d getRotationMatrix(double roll, double pitch, double yaw);
    Eigen::Matrix3d getXRotationMatrix(double roll);
    Eigen::Matrix3d getYRotationMatrix(double pitch);
    Eigen::Matrix3d getZRotationMatrix(double yaw);

    inline double deg2rad(double deg)
    {
      return deg/180.0*M_PI;
    }
  }
}
