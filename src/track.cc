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
    this->initSurfaceInfos();

    GltfExtractor extractor(filename);
    this->initGrounds(extractor);
    this->createCenterline(extractor);
  }

  void Track::initSurfaceInfos()
  {
    s_kGrassSuraface_= 
      {0.f,0.3f,Ground::Type::Grass};
    s_kSandSuraface_= 
      {5.f,0.4f,Ground::Type::Sand};
    s_kGravelSurface_= 
      {3.f,0.4f,Ground::Type::Gravel};
    s_kAsphaltSuraface_= 
      {0.0f,0.1f,Ground::Type::Asphalt};
  }

  void Track::initGrounds(const GltfExtractor& extractor)
  {
    const auto& nodes=extractor.getNodes();

    for(const auto& node : nodes)
    {
      if(node.type!=TINYGLTF_MODE_TRIANGLES)
        continue;

      if(node.vertices.size()==0 || node.indices.size()==0)
        throw Exception(
            Exception::Type::InvalidArgumentError,
            "Ground mesh is empty.");

      // TODO
      grounds.emplace_back(
          node.vertices,
          node.indices,
          node.transform,
          &s_kAsphaltSuraface_,world_);
    }
  }

  void Track::createCenterline(const GltfExtractor& extractor)
  {
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
    gui_->pos=&this->getWorldTransform().getOrigin();
    gui_->filename=this->getFilename().data();
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
