#include "boink/simulators/track/track.h"

#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btTransform.h>

#include "boink/exception.h"
#include "boink/gltf_extractor.h"
#include "boink/gui/track_gui.h"
#include "boink/utility.h"

#include <memory>
#include <vector>

namespace boink
{
  Track::Track(
      std::string_view filename,
      std::shared_ptr<const Weather> weather,
      std::shared_ptr<btDiscreteDynamicsWorld> world)
    :
      world_(world),
      filename_(filename),
      weather_(weather)
  {
    this->initSurfaceInfos();

    GltfExtractor extractor(filename);
    this->initGrounds(extractor);
    this->createCenterline(extractor);
    //this->createRightline(extractor);

    gui_=std::make_shared<TrackGui>(this);
  }

  std::shared_ptr<piksel::GuiObject> Track::getGui() 
  {
    return gui_;
  }

  void Track::initSurfaceInfos()
  {
    btScalar wetness=weather_->getWetness();
    surface_infos_[Ground::Type::Grass]= 
      {0.f,0.3f,wetness,Ground::Type::Grass};
    surface_infos_[Ground::Type::Sand]= 
      {5.f,0.4f,wetness,Ground::Type::Sand};
    surface_infos_[Ground::Type::Gravel]= 
      {3.f,0.4f,wetness,Ground::Type::Gravel};
    surface_infos_[Ground::Type::Asphalt]= 
      {0.0f,0.1f,wetness,Ground::Type::Asphalt};
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
          &surface_infos_[Ground::Type::Asphalt],world_);
    }
  }

  void Track::createCenterline(const GltfExtractor& extractor)
  {
    auto& line_node=extractor.getNode(CENTERLINE_NAME);
    if(line_node.type!=TINYGLTF_MODE_LINE)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Centerline mesh unsupported mode. Use lines mode for centerline mesh.");

    auto& line_vertices=line_node.vertices;
    auto& line_indices=line_node.indices;
    if(line_vertices.size()==0 || line_indices.size()==0)
      throw Exception(
          Exception::Type::InvalidArgumentError,
          "Centerline mesh is empty.");

    std::vector<btVector3> points;
    points.reserve(line_indices.size());

    for(size_t i=0;i<line_indices.size();i+=2)
      points.push_back(line_vertices[line_indices[i]]);

    // Add last point
    points.push_back(
        line_vertices[line_indices[line_indices.size()-1]]);
    
    centerline_=Line(std::move(points));
  }

  void Track::createRightline(const GltfExtractor& extractor)
  {
    const auto& node=extractor.getNode(TRACK_NAME);
    const btVector3 up_dir={0.f,1.f,0.f};
    std::vector<btVector3> points;
    points.reserve(centerline_.getPointsSize());
    
    btVector3 first_point=centerline_.getPoint(0);
    btVector3 second_point=centerline_.getPoint(0+1);
    btVector3 track_dir=second_point-first_point;
    track_dir.normalize();

    // We look for the closet point to centerline ith point which is not
    // colinear to track_dir
    btVector3 not_colinear=track_dir;
    for(
        size_t j=1;
        areColinear(not_colinear,track_dir,1e-2);
        j++)
    {
      size_t index=getIthClosestIndex(node.vertices,first_point,j);
      not_colinear=node.vertices[index]-first_point;
    }

    // Now create orthonormal base
    // we know that not_colinear will be in plane of track
    btScalar in_track_dir=not_colinear.dot(track_dir);
    
    btVector3 right_dir=not_colinear-track_dir*in_track_dir;
    right_dir.normalize();

    // IMPORTANT
    // up_dir is not normal to track plane
    
    // check if it is right or left
    if(right_dir.cross(track_dir).dot(up_dir)<0.f)
      right_dir*=-1.f;

    btVector3 normal_dir=right_dir.cross(track_dir);
    normal_dir.normalize();
    btAssert(normal_dir.dot(up_dir)>0.f);

    ///// Orthonormal base
    /// track_dir
    /// right_dir
    /// normal_dir

    // TODO
    // We have orhonormal basis the only thing left is
    // to move center line by right_dir * sth
    // and check to ceratin limit lets say 100 meters
    // which shift gave the most hits
    // or we simply load left line and right line and
    // then get one closest point which dot product
    // with track_dir is positve and closest one which dot
    // is negative we interpolate and then measure dist
    // to this line and we have right width
    // left symetrically

  }

  void Track::update(btScalar dt)
  {
    (void)dt;

    for(auto& [key,info]:surface_infos_)
      info.wetness=weather_->getWetness();
  }

  void Track::updateRender(Renderer* p_renderer)
  {
    if(p_renderer==nullptr)
      return;
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
