#include "boink/gltf_extractor.h"

#include "boink/exception.h"

#include <sstream>
#include <algorithm>

namespace boink
{
  GltfExtractor::GltfExtractor(std::string_view filename)
    :model_(loadModel(filename))
  {
    if(model_.scenes.size()!=1)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "The glb model must have only one scene.");

    if(model_.defaultScene==-1)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "The glb model must have set default scene value.");

    const tinygltf::Scene& scene=model_.scenes[model_.defaultScene];
    
    for(auto node_index:scene.nodes)
    {
      btTransform transform;
      transform.setIdentity();
      bindNode(model_.nodes[node_index],transform);
    }
  }

  GltfExtractor::Node& GltfExtractor::getNode(std::string_view name)
  {
    auto it=std::find_if(nodes_.begin(),nodes_.end(),
        [=](const Node& node)
        {
          return node.name==name;
        });
    if(it==nodes_.end())
      throw Exception(
          Exception::Type::InvalidArgumentError,
          "Node with a given name was not found.");

    return *it;
  }

  const GltfExtractor::Node& 
    GltfExtractor::getNode(std::string_view name) const
  {
    auto it=std::find_if(nodes_.begin(),nodes_.end(),
        [=](const Node& node)
        {
          return node.name==name;
        });
    if(it==nodes_.end())
      throw Exception(
          Exception::Type::InvalidArgumentError,
          "Node with a given name was not found.");

    return *it;
  }


  void GltfExtractor::bindNode(
      const tinygltf::Node& node, 
      btTransform transform)
  {
    Node new_node;
    new_node.name=node.name;

    transform=getNodeTransform(node)*transform;
    new_node.transform=transform;

    btVector3 scale=getNodeScale(node);
    
    if(node.mesh<0)
      return;

    const tinygltf::Mesh& mesh=model_.meshes[node.mesh];
    
    for(const auto& primitive:mesh.primitives)
    {
      if(primitive.mode!=TINYGLTF_MODE_TRIANGLES &&
          primitive.mode !=TINYGLTF_MODE_LINE)
        throw Exception(
            Exception::Type::UnsupportedFormatError,
            "Triangles and line modes are only supported.");

      new_node.type=primitive.mode;
      
      auto it_pos_index=primitive.attributes.find("POSITION");
      if(it_pos_index==primitive.attributes.end())
        throw Exception(
            Exception::Type::UnsupportedFormatError,
            "POSITION attribiute not found.");
      int pos_index=it_pos_index->second;

      uint32_t base_vertex = static_cast<uint32_t>(new_node.vertices.size());
      loadVertices(model_.accessors.at(pos_index),new_node,scale);
      loadIndices(model_.accessors.at(primitive.indices),new_node,base_vertex);
    }

    nodes_.push_back(std::move(new_node));

    for(auto index:node.children)
      bindNode(model_.nodes[index],transform);
  }

  tinygltf::Model GltfExtractor::loadModel(std::string_view filename)
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

      throw Exception(Exception::Type::IOError,ss.str());
    }

    return model;
  }

  btTransform GltfExtractor::getNodeTransform(
      const tinygltf::Node& node)
  {
    if(node.matrix.size()!=0)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Nodes matrix field must be zero. "
          "Only translation and rotation fields should be used.");

    btVector3 origin(0.f,0.f,0.f);
    btQuaternion rotation(0.f,0.f,0.f,1.f);

    if(node.translation.size()==3)
      origin=btVector3(
          (btScalar)node.translation[0],
          (btScalar)node.translation[1],
          (btScalar)node.translation[2]);
    
    if(node.rotation.size()==4)
      rotation=btQuaternion(
          (btScalar)node.rotation[0],
          (btScalar)node.rotation[1],
          (btScalar)node.rotation[2],
          (btScalar)node.rotation[3]);

    btTransform transform;
    transform.setIdentity();
    transform.setRotation(rotation);
    transform.setOrigin(origin);

    return transform;
  }

  btVector3 GltfExtractor::getNodeScale(const tinygltf::Node& node)
  {
    if(node.matrix.size()!=0)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Nodes matrix field must be zero. "
          "Only scale field should be used.");

    if(node.scale.size()==3)
      return btVector3(
        (btScalar)node.scale[0],
        (btScalar)node.scale[1],
        (btScalar)node.scale[2]);
    else
      return btVector3(btScalar(1.),btScalar(1.),btScalar(1.));
  }

  void GltfExtractor::loadVertices(
      const tinygltf::Accessor& accessor,
      Node& node,
      const btVector3& scale)
  {
    if(accessor.componentType!=TINYGLTF_COMPONENT_TYPE_FLOAT)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Unsupported component type for POSITION attribute.");
    if(accessor.type!=TINYGLTF_TYPE_VEC3)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Unsupported type for POSITION attribue.");

    const auto& buffer_view=
      model_.bufferViews[accessor.bufferView];
    const auto& buffer=
      model_.buffers[buffer_view.buffer];

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
      node.vertices.push_back(btVector3(f[0],f[1],f[2])*scale);
    }
  }

  void GltfExtractor::loadIndices(
      const tinygltf::Accessor& accessor,
      Node& node,
      uint32_t base_vertex)
  {
    if(accessor.componentType!=TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT&&
        accessor.componentType!=TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Unsupported component type for index.");
    if(accessor.type!=TINYGLTF_TYPE_SCALAR)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Unsupported type for index.");

    const auto& buffer_view=
      model_.bufferViews[accessor.bufferView];
    const auto& buffer=
      model_.buffers[buffer_view.buffer];

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
        throw Exception(
            Exception::Type::UnsupportedFormatError,
            "Unsupported data type for index.");
      }

      node.indices.push_back(index + base_vertex);
    }
  }
}
