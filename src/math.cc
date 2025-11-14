#include "boink/utils/math.h"

namespace boink
{
  namespace math
  {
    Eigen::Matrix3d 
      getRotationMatrix(double roll, double pitch, double yaw) 
    {
      return 
        getZRotationMatrix(yaw)*
        getYRotationMatrix(pitch)*
        getXRotationMatrix(roll);
    }

    Eigen::Matrix3d getXRotationMatrix(double roll) 
    {
      Eigen::Matrix3d m;
      double sin=std::sin(roll);
      double cos=std::cos(roll);

      m<<
        1.0,0.0,0.0,
        0.0,cos,-sin,
        0.0,sin,cos;

      return m;
    }
    
    Eigen::Matrix3d getYRotationMatrix(double pitch) 
    {
      Eigen::Matrix3d m;
      double sin=std::sin(pitch);
      double cos=std::cos(pitch);

      m<<
        cos,0.0,sin,
        0.0,1.0,0.0,
        -sin,0.0,cos;

      return m;
    }

    Eigen::Matrix3d getZRotationMatrix(double yaw) 
    {
      Eigen::Matrix3d m;
      double sin=std::sin(yaw);
      double cos=std::cos(yaw);

      m<<
        cos,-sin,0.0,
        sin,cos,0.0,
        0.0,0.0,1.0;

      return m;
    }
  }
}
