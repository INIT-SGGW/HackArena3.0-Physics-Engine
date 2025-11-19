#include <Eigen/Geometry>
#include <iostream>

#include "boink/utils/math.h"

using namespace boink;
using namespace Eigen;

int main() {
  // first 3 are the first COLUMN
  double array[] = {0.99013191915569587,  0, 0.14013844108259077, 0, 1, 0,
                    -0.14013844108259077, 0, 0.99013191915569587};

  Vector3d a = {0, 0, 1.0};
  Vector3d b = {-0.14013844108259077, 0, 0.99013191915569587};

  Matrix3d rot = math::getRotationMatrix(a, b);
  std::cout << rot << std::endl;
  Vector3d angles = rot.canonicalEulerAngles(2, 1, 0);
  std::cout << "Theta_x: " << angles.x() << std::endl;
  std::cout << "Theta_y: " << angles.y() << std::endl;
  std::cout << "Theta_z: " << angles.z() << std::endl;
  Vector3d front = {0.0, 0.0, 2.0};
  Vector3d delta_rot_disp = {0.092776882165181568, 0, 0.34449568810777276};

  std::cout << rot * a << std::endl;
}
