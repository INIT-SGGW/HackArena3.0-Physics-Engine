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
#ifdef MEMORY_TEST
    Kinematics()
      :velocity{},acceleration{}
    {
      std::cout<<"Kinematics Default contructor"<<std::endl;
    }
    Kinematics(const Kinematics& other)
      :velocity(other.velocity),acceleration(other.acceleration)
    {
      std::cout<<"Kinematics Copy contructor"<<std::endl;
    }
    Kinematics(Kinematics&& other) noexcept
      :velocity(std::move(other.velocity)),acceleration(std::move(other.acceleration))
    {
      std::cout<<"Kinematics Move contructor"<<std::endl;
    }
    Kinematics& operator=(const Kinematics& other)
    {
      std::cout<<"Kinematics Copy assigment"<<std::endl;
      velocity=other.velocity;
      acceleration=other.acceleration;

      return *this;
    }
    Kinematics& operator=(Kinematics&& other)
    {
      std::cout<<"Kinematics Copy assigment"<<std::endl;
      velocity=std::move(other.velocity);
      acceleration=std::move(other.acceleration);

      return *this;
    }
    ~Kinematics()
    {
      std::cout<<"Kinematics Destructor"<<std::endl;
    }
#endif
    Eigen::Vector3d velocity{};
    double acceleration{};
  };
}
