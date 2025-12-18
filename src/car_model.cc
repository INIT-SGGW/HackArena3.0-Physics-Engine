#include "boink/components/car_model.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <tiny_gltf.h>

#include <iostream>

namespace boink
{
  //CarModel::CarModel(std::string_view filename)
  //{
  //  
  //  //car_model.front=
  //  //  0.5*(car_model.front_left_wheel-car_model.front_right_wheel)+
  //  //  car_model.front_right_wheel;

  //  //Vector3d temp_rear=
  //  //  0.5*(car_model.rear_left_wheel-car_model.rear_right_wheel)+
  //  //  car_model.rear_right_wheel;
  //  //car_model.direction=car_model.front-temp_rear;
  //  //car_model.direction.normalize();

  //  //Vector3d temp=car_model.rear_left_wheel -car_model.rear_right_wheel;
  //  //
  //  //// It should always point to the top of a car
  //  //car_model.normal=car_model.direction.cross(temp);
  //  //car_model.normal.normalize();
  //}

  glm::mat4 getNodeTransform(const tinygltf::Node& node);
  void bindModelNodes(
      const tinygltf::Model& model,
      const tinygltf::Node& node,
      const glm::mat4& transform);

  CarModel::CarModel(std::string_view filename)
  {
    tinygltf::Model model;

    tinygltf::TinyGLTF loader;
    std::string err;
    std::string warn;

    bool res = loader.LoadBinaryFromFile(&model, &err, &warn, filename.data());

    if (!res)
      throw std::runtime_error("Failed to load model");

    auto scene=model.scenes[model.defaultScene];
    for(auto node_index : scene.nodes)
    {
      bindModelNodes(model,model.nodes[node_index],glm::mat4(1.f));
    }
  }

  void CarModel::bindModelNodes(
      const tinygltf::Model& model,
      const tinygltf::Node& node,
      const glm::mat4& transform)
  {
    glm::mat4 trans=getNodeTransform(node)*transform;

    if(LEFT_REAR_WHEEL_NAME==node.name)
    {
      
    }

    for (int childIndex : node.children)
    {
      std::cout<<"MIALES RACYJE"<<std::endl;
      bindModelNodes(model, model.nodes[childIndex],trans);
    }
  }

  glm::mat4 getNodeTransform(const tinygltf::Node& node)
  {
    glm::mat4 T(1.f);
    glm::mat4 R(1.f);
    glm::mat4 S(1.f);

    if(node.translation.size()==3)
      T=glm::translate(glm::mat4(1.f),{
          node.translation[0],node.translation[1],node.translation[2]});

    if(node.rotation.size()==4)
      R=glm::mat4_cast(glm::quat{
          (float)node.rotation[3],
          (float)node.rotation[0],
          (float)node.rotation[1],
          (float)node.rotation[2]
          });

    if(node.scale.size()==3)
      S=glm::scale(glm::mat4(1.f),{
          node.scale[0],node.scale[1],node.scale[2]
          });

    return (T*R*S);
  }
}
