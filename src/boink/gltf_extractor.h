#pragma once

#include <tiny_gltf.h>

#include <string_view>
#include <vector>

#include <LinearMath/btVector3.h>
#include <LinearMath/btTransform.h>

namespace boink
{
  class GltfExtractor
  {
  public:
    struct Node
    {
      std::string name;
      std::vector<btVector3> vertices;
      std::vector<unsigned int> indices;

      int type;

      btTransform transform;
    };
  public:
    GltfExtractor(std::string_view filename);

    Node& getNode(std::string_view name);
    const Node& getNode(std::string_view name) const;
    std::vector<Node>& getNodes() {return nodes_;}
    const std::vector<Node>& getNodes()const {return nodes_;}
  private:
    void bindNode(const tinygltf::Node& node, btTransform transform);
    void loadVertices(
        const tinygltf::Accessor& accessor,
        Node& node,
        const btVector3& scale);
  void loadIndices(
      const tinygltf::Accessor& accessor,
      Node& node,
      uint32_t base_vertex);
  private:
    static tinygltf::Model loadModel(std::string_view filename);
    static btTransform getNodeTransform(
        const tinygltf::Node& node);
    static btVector3 getNodeScale(
        const tinygltf::Node& node);
  private:
    tinygltf::Model model_;
    std::vector<Node> nodes_;
  };
}
