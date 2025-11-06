#include <eigen3/Eigen/Dense>

#include <iostream>

int main(int argc, char **argv)
{
  // Define a 3D vector
    Eigen::Vector3d v(1.0, 2.0, 3.0);

    // Define a 3x3 matrix
    Eigen::Matrix3d m;
    m << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;

    // Matrix-vector multiplication
    Eigen::Vector3d result = m * v;

    std::cout << "Matrix m:\n" << m << "\n\n";
    std::cout << "Vector v:\n" << v << "\n\n";
    std::cout << "Result m * v:\n" << result << std::endl;

    return 0;
}
