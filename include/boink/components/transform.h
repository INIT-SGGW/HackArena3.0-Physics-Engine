#pragma once

#include <Eigen/Core>

#ifdef MEMORY_TEST
#include <iostream>
#endif
namespace boink
{
  /**
   * @brief Represents an entity's spatial transformation.
   */
  struct Transform
  {
#ifdef MEMORY_TEST
    Transform()
      :position{},rotation{Eigen::Matrix3d::Identity()}
    {
      std::cout<<"Transform Default contructor"<<std::endl;
    }
    Transform(const Transform& other)
      :position(other.position),rotation(other.rotation)
    {
      std::cout<<"Transform Copy contructor"<<std::endl;
    }
    Transform(Transform&& other) noexcept
      :position(std::move(other.position)),rotation(std::move(other.rotation))
    {
      std::cout<<"Transform Move contructor"<<std::endl;
    }
    Transform& operator=(const Transform& other)
    {
      std::cout<<"Transform Copy assigment"<<std::endl;
      position=other.position;
      rotation=other.rotation;

      return *this;
    }
    Transform& operator=(Transform&& other)
    {
      std::cout<<"Transform Copy assigment"<<std::endl;
      position=std::move(other.position);
      rotation=std::move(other.rotation);

      return *this;
    }
    ~Transform()
    {
      std::cout<<"Transform Destructor"<<std::endl;
    }
#endif
    Eigen::Vector3d position{};
    Eigen::Matrix3d rotation=Eigen::Matrix3d::Identity();
  };
}
