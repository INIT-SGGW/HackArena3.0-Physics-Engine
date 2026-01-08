#pragma once

#include <glm/geometric.hpp>
#include <LinearMath/btVector3.h>
#include <piksel/model.hh>

namespace boink
{
  /**
   * @brief Represents model of a car which is constant during simulation.
   */
  class CarModel
  {
  public:
    CarModel(
        double mass,
        double radius,
        double steer_angle_deg,
        std::string_view filepath);

    const btVector3& getRearLeftWheel() const {return rear_left_wheel_;}
    const btVector3& getRearRightWheel() const {return rear_right_wheel_;}
    const btVector3& getFrontLeftWheel() const {return front_left_wheel_;}
    const btVector3& getFrontRightWheel() const {return front_right_wheel_;}

    double getMaxSteerAngleDegrees() const {return max_steer_angle_deg_;}
    double getWheelRadius() const {return radius_;}

    double getMass() const {return mass_;}

    const piksel::Model& getModel() const {return model_;}
  public:
    static constexpr std::string_view BODY_NAME="Cylinder.002";
    static constexpr std::string_view REAR_LEFT_WHEEL_NAME="Cylinder.005";
    static constexpr std::string_view REAR_RIGHT_WHEEL_NAME="Cylinder.004";
    static constexpr std::string_view FRONT_LEFT_WHEEL_NAME="Cylinder.003";
    static constexpr std::string_view FRONT_RIGHT_WHEEL_NAME="Cylinder.007";

  private:
    piksel::Model model_;
    double mass_;

    btVector3 rear_left_wheel_{};
    btVector3 rear_right_wheel_{};
    btVector3 front_left_wheel_{};
    btVector3 front_right_wheel_{};
    double max_steer_angle_deg_{};
    double radius_;

    btVector3 front_{};
    btVector3 direction_{}; // Normalized vector
    btVector3 normal_{}; // Normalized. Points to the top of a car 
  };
}
