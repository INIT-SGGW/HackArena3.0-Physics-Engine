#pragma once

#include <piksel/mesh.hh>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <LinearMath/btVector3.h>
#include <LinearMath/btTransform.h>

#include <piksel/object.hh>

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
      mutable std::shared_ptr<piksel::Mesh> piksel_mesh;

      btTransform transform;
    };
  public:
    VehicleMesh(std::string_view filename);

    const Element& getChassis() const;
    const Element& getWheel(WheelPosition wheel) const;
    std::shared_ptr<const piksel::Mesh> getChassisPikselMesh() const;
    std::shared_ptr<const piksel::Mesh> 
      getWheelPikselMesh(WheelPosition pos) const;

    btTransform getLocalWheelTransform(WheelPosition wheel) const;

    btVector3 getLocalForward() const {return local_forward_;}
    btVector3 getLocalLeft() const {return local_left_;}
    btVector3 getLocalUp() const {return local_up_;}
  private:
    static std::shared_ptr<piksel::Mesh> createPikselMesh(
        const std::vector<btVector3>& vertices,
        const std::vector<unsigned int>& indices);
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

    btVector3 local_left_;
    btVector3 local_up_;
    btVector3 local_forward_;
  };
}
