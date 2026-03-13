#include "boink/simulators/vehicle/vehicle_mesh.h"

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <iterator>
#include <piksel/object.hh>

#include "boink/gltf_extractor.h"
#include "boink/utility.h"
#include "boink/constants.h"
#include "boink/exception.h"

#include <algorithm>
#include <vector>

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
        nullptr,
        std::move(node.transform)};
      wheels_.insert(
          {pair.second,std::move(element)});
    }

    const auto& node=extractor.getNode(CHASSIS_NAME);
    chassis_={
        std::move(node.vertices),
        std::move(node.indices),
        nullptr,
        std::move(node.transform)};

    btVector3 rear_right=getWheel(WheelPosition::RearRight).transform.getOrigin();
    btVector3 rear_left=getWheel(WheelPosition::RearLeft).transform.getOrigin();
    btVector3 front_right=getWheel(WheelPosition::FrontRight).transform.getOrigin();
    btVector3 front_left=getWheel(WheelPosition::FrontLeft).transform.getOrigin();

    btVector3 rear_midpoint =
      (rear_left+rear_right)*0.5f;

    btVector3 front_midpoint =
      (front_left+front_right)*0.5f;

    local_forward_=(front_midpoint-rear_midpoint).normalize();
    local_left_=(rear_left-rear_right).normalize();
    local_up_=local_forward_.cross(local_left_).normalize();

    if((local_forward_-g_Forward).length2()>g_Epsilon)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Vehicle model forward must be equal to g_Forward");
    if((local_left_-g_Left).length2()>g_Epsilon)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Vehicle model left must be equal to g_Left");
    if((local_up_-g_Up).length2()>g_Epsilon)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Vehicle model up must be equal to g_Up");
  }

  std::shared_ptr<const piksel::Mesh> 
    VehicleMesh::getChassisPikselMesh() const 
  {
    if(!chassis_.piksel_mesh)
      chassis_.piksel_mesh=
        createPikselMesh(chassis_.vertices,chassis_.indices);

    return chassis_.piksel_mesh;
  }

  std::shared_ptr<const piksel::Mesh> 
   VehicleMesh:: getWheelPikselMesh(WheelPosition pos) const
  {
    auto& wheel=wheels_.at(pos);
    if(!wheel.piksel_mesh)
      wheel.piksel_mesh=createPikselMesh(wheel.vertices,wheel.indices);

    return wheel.piksel_mesh;
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

  std::shared_ptr<piksel::Mesh> VehicleMesh::createPikselMesh(
      const std::vector<btVector3>& vertices,
      const std::vector<unsigned int>& indices)
  {
    // Create chassis piksel mesh
    std::vector<piksel::Mesh::Vertex> piksel_vertices;
    piksel_vertices.reserve(vertices.size());

    std::transform(vertices.begin(),vertices.end(),
        std::back_inserter(piksel_vertices),
        [](const btVector3& vec)
        {
          return piksel::Mesh::Vertex{math::bt2glm(vec)};
        });
    
    return std::make_shared<piksel::Mesh>(piksel_vertices,indices);
  }
}
