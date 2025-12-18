#pragma once

#include <string_view>

#include <glm/glm.hpp>
#include <tiny_gltf.h>

#include <Eigen/Core>

#ifdef MEMORY_TEST
#include <iostream>
#endif
namespace boink
{
  /**
   * @brief Represents model of a car which is constant during simulation.
   */
  struct CarModel
  {
    std::string_view LEFT_REAR_WHEEL_NAME="Cylinder.005";
    std::string_view RIGHT_REAR_WHEEL_NAME="Cylinder.004";
    std::string_view LEFT_FRONT_WHEEL_NAME="Cylinder.003";
    std::string_view RIGHT_FRONT_WHEEL_NAME="Cylinder.007";
    std::string_view CHASSIS_NAME="Cylinder.002";
#ifdef MEMORY_TEST
    CarModel()
    {
        std::cout << "CarModel default ctor   this=" << this << std::endl;
    }

    CarModel(const CarModel& other)
        : rear_left_wheel(other.rear_left_wheel),
          rear_right_wheel(other.rear_right_wheel),
          front_left_wheel(other.front_left_wheel),
          front_right_wheel(other.front_right_wheel),
          max_steer_angle_deg(other.max_steer_angle_deg),
          front(other.front),
          direction(other.direction),
          normal(other.normal)
    {
        std::cout << "CarModel copy ctor      this=" << this
                  << "  from=" << &other << std::endl;
    }

    CarModel(CarModel&& other) noexcept
        : rear_left_wheel(std::move(other.rear_left_wheel)),
          rear_right_wheel(std::move(other.rear_right_wheel)),
          front_left_wheel(std::move(other.front_left_wheel)),
          front_right_wheel(std::move(other.front_right_wheel)),
          max_steer_angle_deg(other.max_steer_angle_deg),
          front(std::move(other.front)),
          direction(std::move(other.direction)),
          normal(std::move(other.normal))
    {
        std::cout << "CarModel move ctor      this=" << this
                  << "  from=" << &other << std::endl;
    }

    CarModel& operator=(const CarModel& other)
    {
        if (this != &other)
        {
            std::cout << "CarModel copy assign    this=" << this
                      << "  from=" << &other << std::endl;

            rear_left_wheel   = other.rear_left_wheel;
            rear_right_wheel  = other.rear_right_wheel;
            front_left_wheel  = other.front_left_wheel;
            front_right_wheel = other.front_right_wheel;
            max_steer_angle_deg = other.max_steer_angle_deg;

            front = other.front;
            direction = other.direction;
            normal = other.normal;
        }

        return *this;
    }

    CarModel& operator=(CarModel&& other) noexcept
    {
        if (this != &other)
        {
            std::cout << "CarModel move assign    this=" << this
                      << "  from=" << &other << std::endl;

            rear_left_wheel   = std::move(other.rear_left_wheel);
            rear_right_wheel  = std::move(other.rear_right_wheel);
            front_left_wheel  = std::move(other.front_left_wheel);
            front_right_wheel = std::move(other.front_right_wheel);

            max_steer_angle_deg = other.max_steer_angle_deg;

            front = std::move(other.front);
            direction = std::move(other.direction);
            normal = std::move(other.normal);
        }

        return *this;
    }

    ~CarModel()
    {
        std::cout << "CarModel dtor           this=" << this << std::endl;
    }
#endif

    CarModel(std::string_view filename);

    // TODO
    // Make it vars const and create ctor
    Eigen::Vector3d rear_left_wheel{};
    Eigen::Vector3d rear_right_wheel{};
    Eigen::Vector3d front_left_wheel{};
    Eigen::Vector3d front_right_wheel{};
    double max_steer_angle_deg{};

    Eigen::Vector3d front{};
    Eigen::Vector3d direction{}; // Normalized vector
    Eigen::Vector3d normal{}; // Normalized. Points to the top of a car 
  private:
    void bindModelNodes(
        const tinygltf::Model& model,
        const tinygltf::Node& node,
        const glm::mat4& transform);
};
}
