#include "boink/simulation/vehicle_mesh.h"

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace boink
{
  VehicleMesh::VehicleMesh(std::string_view filename)
  {
    tinygltf::Model model=loadModel(filename);

    assert(model.scenes.size()==1);
    const tinygltf::Scene& scene=model.scenes[model.defaultScene];
    
    for(auto node_index:scene.nodes)
    {
      btTransform transform;
      transform.setIdentity();
      bindNode(model,model.nodes[node_index],transform);
    }
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

  void VehicleMesh::bindNode(
      const tinygltf::Model& model,
      const tinygltf::Node& node, 
      btTransform transform)
  {
    if(!isNamePresent(node.name))
      return;
    Element& element=
      CHASSIS_NAME==node.name?chassis_:wheels_[s_wheel_names_.at(node.name)];

    transform=getNodeTransform(node)*transform;
    element.transform=transform;

    btVector3 scale=getNodeScale(node);
    
    if(node.mesh<0)
      throw std::runtime_error("Node does not have mesh");
    const tinygltf::Mesh& mesh=model.meshes[node.mesh];
    
    for(const auto& primitive:mesh.primitives)
    {
      if(primitive.mode!=TINYGLTF_MODE_TRIANGLES)
        throw std::runtime_error("Only triangles mode");
      
      auto it_pos_index=primitive.attributes.find("POSITION");
      if(it_pos_index==primitive.attributes.end())
        throw std::runtime_error("POSITION attribiute not found");
      int pos_index=it_pos_index->second;

      uint32_t base_vertex = static_cast<uint32_t>(element.vertices.size());
      loadVertices(model.accessors.at(pos_index),model,element,scale);
      loadIndices(model.accessors.at(primitive.indices),model,element,base_vertex);
    }

    for(auto index:node.children)
      bindNode(model,model.nodes[index],transform);
  }

  tinygltf::Model VehicleMesh::loadModel(std::string_view filename)
  {
    tinygltf::TinyGLTF loader;
    tinygltf::Model model;
    std::string err;
    std::string warn;

    bool res=loader.LoadBinaryFromFile(
        &model,&err,&warn,std::string(filename));

    if(!res)
    {
      std::ostringstream ss;
      ss<<"Failed to load glb model."<<std::endl;
      if(!warn.empty())
        ss<<"Warning: "<<warn<<std::endl;
      if(!err.empty())
        ss<<"Error: "<<err<<std::endl;

      throw std::runtime_error(ss.str());
    }

    return model;
  }

  bool VehicleMesh::isNamePresent(std::string_view name)
  {
    if(s_wheel_names_.find(name)!=s_wheel_names_.end())
      return true;
    
    return name==CHASSIS_NAME;
  }

  btTransform VehicleMesh::getNodeTransform(
      const tinygltf::Node& node)
  {
    assert(node.matrix.size()==0);

    btVector3 origin(0.f,0.f,0.f);
    btQuaternion rotation(0.f,0.f,0.f,1.f);

    if(node.translation.size()==3)
      origin=btVector3(
          node.translation[0],
          node.translation[1],
          node.translation[2]);
    
    if(node.rotation.size()==4)
      rotation=btQuaternion(
          node.rotation[0],
          node.rotation[1],
          node.rotation[2],
          node.rotation[3]);

    btTransform transform;
    transform.setIdentity();
    transform.setRotation(rotation);
    transform.setOrigin(origin);

    return transform;
  }

  btVector3 VehicleMesh::getNodeScale(const tinygltf::Node& node)
  {
    if(node.scale.size()==3)
      return btVector3(node.scale[0],node.scale[1],node.scale[2]);
    else
      return btVector3(1.f,1.f,1.f);
  }

  void VehicleMesh::loadVertices(
      const tinygltf::Accessor accessor,
      const tinygltf::Model& model,
      Element& element,
      const btVector3& scale)
  {
    if(accessor.componentType!=TINYGLTF_COMPONENT_TYPE_FLOAT)
      throw std::runtime_error("Unsupported component type for position");
    if(accessor.type!=TINYGLTF_TYPE_VEC3)
      throw std::runtime_error("Unsupported type for position");

    const auto& buffer_view=
      model.bufferViews[accessor.bufferView];
    const auto& buffer=
      model.buffers[buffer_view.buffer];

    const unsigned char* p_data=
      buffer.data.data()+
      buffer_view.byteOffset+
      accessor.byteOffset;

    auto stride=accessor.ByteStride(buffer_view);
    if (stride == 0) stride = 3 * sizeof(float); // VEC3 tightly packed

    for(size_t i=0;i<accessor.count;i++)
    {
      const auto* pos_data=&p_data[i*stride];
      const float* f=reinterpret_cast<const float*>(pos_data);
      
      // Bake scale into vertices
      element.vertices.push_back(btVector3(f[0],f[1],f[2])*scale);
    }
  }

  void VehicleMesh::loadIndices(
      const tinygltf::Accessor accessor,
      const tinygltf::Model& model,
      Element& element,
      uint32_t base_vertex)
  {
    if(accessor.componentType!=TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT&&
        accessor.componentType!=TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT)
      throw std::runtime_error("Unsupported component type for index");
    if(accessor.type!=TINYGLTF_TYPE_SCALAR)
      throw std::runtime_error("Unsupported type for index");

    const auto& buffer_view=
      model.bufferViews[accessor.bufferView];
    const auto& buffer=
      model.buffers[buffer_view.buffer];

    const unsigned char* p_data=
      buffer.data.data()+
      buffer_view.byteOffset+
      accessor.byteOffset;

    auto stride=accessor.ByteStride(buffer_view);
    if (stride == 0)
        stride = (accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT)
          ? 2 : 4;

    for(size_t i=0;i<accessor.count;i++)
    {
      const auto* idx_data=&p_data[i*stride];

      uint32_t index = 0;

      switch (accessor.componentType)
      {
      case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
        index = *reinterpret_cast<const uint16_t*>(idx_data);
        break;
      case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
        index = *reinterpret_cast<const uint32_t*>(idx_data);
        break;
      default:
        throw std::runtime_error("Unsupported index type");
      }

      element.indices.push_back(index + base_vertex);
    }
  }
}
