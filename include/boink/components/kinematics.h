#pragma once

#include <Eigen/Core>

#ifdef MEMORY_TEST
#include <iostream>
#endif

namespace boink
{
/**
 * @brief Describes body motion.
 */
struct Kinematics
{
  // TODO: move chassis parameters to new chassis component
#ifdef MEMORY_TEST
  Kinematics() : velocity{}, acceleration{} { std::cout << "Kinematics Default contructor" << std::endl; }
  Kinematics(const Kinematics& other) : velocity(other.velocity), acceleration(other.acceleration)
  {
    std::cout << "Kinematics Copy contructor" << std::endl;
  }
  Kinematics(Kinematics&& other) noexcept
      : velocity(std::move(other.velocity)), acceleration(std::move(other.acceleration))
  {
    std::cout << "Kinematics Move contructor" << std::endl;
  }
  Kinematics& operator=(const Kinematics& other)
  {
    std::cout << "Kinematics Copy assigment" << std::endl;
    velocity = other.velocity;
    acceleration = other.acceleration;

    return *this;
  }
  Kinematics& operator=(Kinematics&& other)
  {
    std::cout << "Kinematics Copy assigment" << std::endl;
    velocity = std::move(other.velocity);
    acceleration = std::move(other.acceleration);

    return *this;
  }
  ~Kinematics() { std::cout << "Kinematics Destructor" << std::endl; }
#endif
  Kinematics() : dynamic_front_weight(kStationaryFrontWeight), dynamic_rear_weight(kStationaryRearWeight) {}

  float dynamic_front_weight;
  float dynamic_rear_weight;

  Eigen::Vector3d velocity{};
  float traction_force{};

  // distance between centers of left and right wheels on front axle in [m]
  static constexpr float kFrontTrackWidth = 1.47f;
  // distance between centers of left and right wheels on rear axle in [m]
  static constexpr float kRearTrackWidth = 1.42f;

  // distance between front and rear axles in [m]
  static constexpr float kWheelBase = 3.05f;
  static constexpr float kCoG_to_rear = 1.40f;
  static constexpr float kCoGHeight = 0.25f;

  static constexpr float kMass = 650.0f;
  static constexpr float kWeight = kMass * 9.81f;
  static constexpr float kStationaryFrontWeight = kCoG_to_rear / kWheelBase * kWeight;
  static constexpr float kStationaryRearWeight = (kWheelBase - kCoG_to_rear) / kWheelBase * kWeight;
};
}  // namespace boink