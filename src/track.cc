#include "boink/simulators/track/track.h"

#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btTransform.h>

#include "boink/exception.h"
#include "boink/gltf_extractor.h"

#include <memory>
#include <vector>

namespace boink
{
  Track::Track(std::string_view filename,
      std::shared_ptr<btDiscreteDynamicsWorld> world)
    :
      world_(world),
      filename_(filename),
      gui_(std::make_shared<TrackGui>())
  {
    GltfExtractor extractor(filename);
    auto& nodes=extractor.getNodes();

    for(auto& node : nodes)
    {
      if(node.type!=TINYGLTF_MODE_TRIANGLES)
        continue;

      if(node.vertices.size()==0 || node.indices.size()==0)
        throw Exception(
            Exception::Type::InvalidArgumentError,
            "Ground mesh is empty.");

      grounds.emplace_back(
          node.vertices,
          node.indices,
          node.transform,
          Ground::Type::Tarmac,world_);
    }

    // Load centerline
    auto& centerline_node=extractor.getNode(CENTERLINE_NAME);
    if(centerline_node.type!=TINYGLTF_MODE_LINE)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Centerline mesh unsupported mode. Use lines mode for centerline mesh.");

    auto& centerline_vertices=centerline_node.vertices;
    auto& centerline_indices=centerline_node.indices;
    if(centerline_vertices.size()==0 || centerline_indices.size()==0)
      throw Exception(
          Exception::Type::InvalidArgumentError,
          "Centerline mesh is empty.");

    std::vector<btVector3> points;
    points.reserve(centerline_indices.size());

    for(size_t i=0;i<centerline_indices.size();i+=2)
      points.push_back(centerline_vertices[centerline_indices[i]]);

    // Add last point
    points.push_back(
        centerline_vertices[centerline_indices[centerline_indices.size()-1]]);
    
    centerline_=Centerline(std::move(points));
  }

  void Track::update(btScalar dt)
  {
    (void)dt;
  }

  void Track::updateRender(Renderer* p_renderer)
  {
    assert(p_renderer!=nullptr);

    this->updateGui();
  }

  void Track::updateGui()
  {
    gui_->pos=this->getWorldTransform().getOrigin();
    gui_->filename=this->getFilename();
  }

  void Track::setWorldTransform(const btTransform& transform)
  {
    transform_=transform;
    for(auto& ground:grounds)
    {
      btTransform new_transform=transform_*ground.getModelTransform();
      ground.setWorldTransform(new_transform);
    }
  }
}
