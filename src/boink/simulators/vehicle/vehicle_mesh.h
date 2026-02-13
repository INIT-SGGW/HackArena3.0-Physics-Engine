#pragma once

#include <string_view>
#include <unordered_map>
#include <vector>

#include <LinearMath/btVector3.h>
#include <LinearMath/btTransform.h>

#include "boink/simulators/vehicle/wheel_position.h"

namespace boink
{
  class VehicleMesh
  {
  public:
    struct Element
    {
      std::vector<btVector3> vertices;
      std::vector<unsigned int> indices;

      btTransform transform;
    };
  public:
    VehicleMesh(std::string_view filename);

    const Element& getChassis() const;
    const Element& getWheel(WheelPosition wheel) const;

    btTransform getLocalWheelTransform(WheelPosition wheel) const;
  private:
    static constexpr std::string_view CHASSIS_NAME="Cylinder.002";
    inline static const std::unordered_map<std::string_view, WheelPosition> 
      s_wheel_names_{
        {"Cylinder.004", WheelPosition::RearRight},
        {"Cylinder.005", WheelPosition::RearLeft},
        {"Cylinder.007", WheelPosition::FrontRight},
        {"Cylinder.003", WheelPosition::FrontLeft},
};
  private:
    std::unordered_map<WheelPosition,Element> wheels_;
    Element chassis_;
  };
}
