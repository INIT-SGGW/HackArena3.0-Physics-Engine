#pragma once

#include <Eigen/Core>

#include <numbers>

namespace boink
{
  namespace math
  {
    // TODO
    // All of this matrices are subject to an effect called gimbal lock
    // usage of quatrions is preffered

    /**
     * @brief Computes a 3D rotation matrix.
     * 
     * The rotation is applied in the order: X-axis, then Y-axis, then Z-axis.
     *
     * @param theta_x Rotation angle about the X-axis (radians).
     * @param theta_y Rotation angle about the Y-axis (radians).
     * @param theta_z Rotation angle about the Z-axis (radians).
     *
     * @return Rotation matrix.
     */
    Eigen::Matrix3d getRotationMatrix(double theta_x, double theta_y, double theta_z);

    /**
     * @brief Computes a rotation matrix about the X-axis.
     *
     * @param theta_x Rotation angle (radians).
     *
     * @return Rotation matrix for X-axis rotation.
     */
    Eigen::Matrix3d getXRotationMatrix(double theta_x);

    /**
     * @brief Computes a rotation matrix about the Y-axis.
     *
     * @param theta_y Rotation angle (radians).
     *
     * @return Rotation matrix for Y-axis rotation.
     */
    Eigen::Matrix3d getYRotationMatrix(double theta_y);

    /**
     * @brief Computes a rotation matrix about the Z-axis.
     *
     * @param theta_z Rotation angle (radians).
     *
     * @return Rotation matrix for Z-axis rotation.
     */
    Eigen::Matrix3d getZRotationMatrix(double theta_z);

    /**
     * @brief Computes a rotation matrix that rotates vector a to align with vector b.
     *
     * Both input vectors must be normalized. The resulting rotation matrix R satisfies:
     * R * a_norm ≈ b_norm
     *
     * @param a_norm Normalized starting vector.
     * @param b_norm Normalized target vector.
     *
     * @return Rotation matrix.
     */
    Eigen::Matrix3d getRotationMatrix(
        const Eigen::Vector3d& a_norm,
        const Eigen::Vector3d& b_norm);

    /**
     * @brief Computes a rotation matrix using Rodrigues' rotation formula.
     *
     * Given the sine and cosine of the rotation angle and the normalized rotation axis,
     * this function returns the corresponding 3×3 rotation matrix.
     *
     * @param sin_theta Sine of the rotation angle.
     * @param cos_theta Cosine of the rotation angle.
     * @param axis_norm Normalized rotation axis vector.
     *
     * @return Rotation matrix representing the rotation about axis_norm.
     */
    Eigen::Matrix3d getRodriguesRotationMatrix(
        double sin_theta,
        double cos_theta,
        const Eigen::Vector3d& axis_norm);

    /**
     * @brief Converts an angle from degrees to radians.
     *
     * @param deg Angle in degrees.
     *
     * @return Angle in radians.
     */
    inline double deg2rad(double deg)
    {
      return deg/180.0*std::numbers::pi;
    }
  }
}
