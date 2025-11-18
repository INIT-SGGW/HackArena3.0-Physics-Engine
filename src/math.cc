#include "boink/utils/math.h"
#include <Eigen/Geometry>

#include <cmath>

using namespace Eigen;

namespace boink
{
  namespace math
  {
    Eigen::Matrix3d 
      getRotationMatrix(double theta_x, double theta_y, double theta_z) 
    {
      // TODO
      // To optimize we can pass already sin_theta and cos_theta.
      return 
        getZRotationMatrix(theta_z)*
        getYRotationMatrix(theta_y)*
        getXRotationMatrix(theta_x);
    }

    Eigen::Matrix3d getXRotationMatrix(double theta_x) 
    {
      Eigen::Matrix3d m;
      double sin=std::sin(theta_x);
      double cos=std::cos(theta_x);

      m<<
        1.0,0.0,0.0,
        0.0,cos,-sin,
        0.0,sin,cos;

      return m;
    }
    
    Eigen::Matrix3d getYRotationMatrix(double theta_y) 
    {
      Eigen::Matrix3d m;
      double sin=std::sin(theta_y);
      double cos=std::cos(theta_y);

      m<<
        cos,0.0,sin,
        0.0,1.0,0.0,
        -sin,0.0,cos;

      return m;
    }

    Eigen::Matrix3d getZRotationMatrix(double theta_z) 
    {
      Eigen::Matrix3d m;
      double sin=std::sin(theta_z);
      double cos=std::cos(theta_z);

      m<<
        cos,-sin,0.0,
        sin,cos,0.0,
        0.0,0.0,1.0;

      return m;
    }

    Eigen::Matrix3d getRotationMatrix(
        const Eigen::Vector3d& a_norm,
        const Eigen::Vector3d& b_norm)
    {
      double cos_theta=a_norm.dot(b_norm);

      Vector3d axis=a_norm.cross(b_norm);
      double sin_theta=axis.norm();

      return getRodriguesRotationMatrix(sin_theta,cos_theta,axis.normalized());
    }
    
    Eigen::Matrix3d getRodriguesRotationMatrix(
        double sin_theta,
        double cos_theta,
        const Eigen::Vector3d& axis_norm)
    {
      Matrix3d k {
        {0.0,-axis_norm.z(),axis_norm.y()},
        {axis_norm.z(),0.0,-axis_norm.x()},
        {-axis_norm.y(),axis_norm.x(),0.0}
      };

      return Matrix3d::Identity()+k*sin_theta+k*k*(1-cos_theta);
    }
  }
}
