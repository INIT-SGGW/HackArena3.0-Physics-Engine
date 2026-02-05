#pragma once

#ifdef MEMORY_TEST
#include <iostream>
#endif

namespace boink
{
struct CarInput
{
#ifdef MEMORY_TEST
  CarInput() : throttle{}, brake{}, steer_angle{} { std::cout << "CarInput Default contructor" << std::endl; }
  CarInput(const CarInput& other) : throttle(other.throttle), brake(other.brake), steer_angle(other.steer_angle)
  {
    std::cout << "CarInput Copy contructor" << std::endl;
  }
  CarInput(CarInput&& other) noexcept
      : throttle(std::move(other.throttle)), brake(std::move(other.brake)), steer_angle(std::move(other.steer_angle))
  {
    std::cout << "CarInput Move contructor" << std::endl;
  }
  CarInput& operator=(const CarInput& other)
  {
    std::cout << "CarInput Copy assigment" << std::endl;
    throttle = other.throttle;
    brake = other.brake;
    steer_angle = other.steer_angle;

    return *this;
  }
  CarInput& operator=(CarInput&& other) noexcept
  {
    std::cout << "CarInput Move assigment" << std::endl;
    throttle = std::move(other.throttle);
    brake = std::move(other.brake);
    steer_angle = std::move(other.steer_angle);

    return *this;
  }
  ~CarInput() { std::cout << "CarInput Destructor" << std::endl; }
#endif
  double throttle{};
  double brake{};
  double steer_angle{};
  bool gear_up = false;
  bool gear_down = false;
};
}  // namespace boink
