#pragma once

#include <string_view>
#include <tiny_gltf.h>
#include <unordered_map>
#include <vector>
#include <LinearMath/btVector3.h>
#include <LinearMath/btTransform.h>

namespace boink
{
  class VehicleModel
  {
  public:
    struct Element
    {
      std::vector<btVector3> vertices;
      std::vector<unsigned int> indices;

      btTransform transform;
    };
    enum class Wheel
    {
      RearLeft,
      RearRight,
      FrontLeft,
      FrontRight,
    };
  public:
    VehicleModel(std::string_view filename);

    const Element& getChassis() const;
    const Element& getWheel(Wheel wheel) const;
  private:
    void bindNode(
        const tinygltf::Model& model,
        const tinygltf::Node& node, 
        btTransform transform);
  private:
    static tinygltf::Model loadModel(std::string_view filename);
    static bool isNamePresent(std::string_view name);
    static btTransform getNodeTransform(const tinygltf::Node& node);
    static btVector3 getNodeScale(const tinygltf::Node& node);

    static void loadVertices(
        const tinygltf::Accessor accessor,
        const tinygltf::Model& model,
        Element& element,
        const btVector3& scale);
    static void loadIndices(
        const tinygltf::Accessor accessor,
        const tinygltf::Model& model,
        Element& element,
        uint32_t base_vertex);
  private:
    static constexpr std::string_view CHASSIS_NAME="Cylinder.002";
    inline static const std::unordered_map<std::string_view, Wheel> s_wheel_names_{
        {"Cylinder.004", Wheel::RearRight},
        {"Cylinder.005", Wheel::RearLeft},
        {"Cylinder.007", Wheel::FrontRight},
        {"Cylinder.003", Wheel::FrontLeft},
};
  private:
    std::unordered_map<Wheel,Element> wheels_;
    Element chassis_;
  };
}
