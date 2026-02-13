#include "boink/simulators/vehicle/vehicle_mesh.h"

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

#include "boink/gltf_extractor.h"

namespace boink
{
  VehicleMesh::VehicleMesh(std::string_view filename)
  {
    GltfExtractor extractor(filename);
    
    for( const auto& pair:s_wheel_names_)
    {
      const auto& node=extractor.getNode(pair.first);
      Element element={
        std::move(node.vertices),
        std::move(node.indices),
        std::move(node.transform)};
      wheels_.insert(
          {pair.second,std::move(element)});
    }

    const auto& node=extractor.getNode(CHASSIS_NAME);
    chassis_={
        std::move(node.vertices),
        std::move(node.indices),
        std::move(node.transform)};
  }

  const VehicleMesh::Element& VehicleMesh::getChassis() const
  {
    return chassis_;
  }

  const VehicleMesh::Element& VehicleMesh::getWheel(WheelPosition wheel) const
  {
    return wheels_.at(wheel);
  }

  btTransform VehicleMesh::getLocalWheelTransform(WheelPosition wheel) const
  {
    return chassis_.transform.inverse()*wheels_.at(wheel).transform;
  }
}
