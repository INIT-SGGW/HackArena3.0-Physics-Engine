#include "boink/simulators/track/track.h"

#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btScalar.h>
#include <LinearMath/btTransform.h>

#include "boink/exception.h"
#include "boink/gltf_extractor.h"
#include "boink/gui/track_gui.h"

#include <memory>
#include <sstream>
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
    this->createLines(extractor);

    this->createTrackData();
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
      if(node.name!=TRACK_NAME)
        continue;
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

  void Track::createLines(const GltfExtractor& extractor)
  {
    Track::createLine(extractor,centerline_,CENTERLINE_NAME);
    Track::createLine(extractor,rightline_,RIGHTLINE_NAME);
    Track::createLine(extractor,leftline_,LEFTLINE_NAME);

    // Check if centerline should be reveresed

    auto center_point=centerline_.getPoint(0);
    auto next_center_point=centerline_.getPoint(1);

    auto dir=next_center_point-center_point;

    auto right_point=rightline_.getPoint( rightline_.getClosestIndex(center_point).first);
    auto right=right_point-center_point;

    auto normal=right.cross(dir);

    if(normal.dot(s_kUp)<0)
      centerline_.reverse();
  }

  void Track::createLine(
      const GltfExtractor& extractor, Line& line, std::string_view name)
  {
    auto& line_node=extractor.getNode(name);
    if(line_node.type!=TINYGLTF_MODE_LINE)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Line mesh unsupported mode. Use lines mode for line mesh.");

    auto& line_vertices=line_node.vertices;
    auto& line_indices=line_node.indices;
    if(line_vertices.size()==0 || line_indices.size()==0)
      throw Exception(
          Exception::Type::InvalidArgumentError,
          "Line mesh is empty.");

    std::vector<btVector3> points;
    points.reserve(line_indices.size());

    for(size_t i=0;i<line_indices.size();i+=2)
      points.push_back(line_vertices[line_indices[i]]);

    // Add last point
    points.push_back(
        line_vertices[line_indices[line_indices.size()-1]]);
    
    line=Line(std::move(points));
  }

  void Track::createTrackData()
  {
    track_data_.reserve(centerline_.getPointsSize());

    btAssert(centerline_.getPointsSize()>2);
    btAssert(rightline_.getPointsSize()>2);
    btAssert(leftline_.getPointsSize()>2);

    for(size_t i=0;i<centerline_.getPointsSize()-1;i++)
      track_data_.push_back(this->generateSampleTrackData(i));

    // For last element
    track_data_.push_back(
        this->generateSampleTrackData(centerline_.getPointsSize()-1));

    // Calc curvature jebana
    for(size_t i=0;i<track_data_.size();i++)
    {
      size_t prev=(i-1+track_data_.size())%track_data_.size();
      size_t next=(i+1)%track_data_.size();

      const auto& sample_prev=track_data_[prev];
      const auto& sample_next=track_data_[next];
      btVector3 dT=sample_next.tangent-sample_prev.tangent;
      btScalar ds=sample_next.coverage-sample_prev.coverage;

      btVector3 dTds=dT/ds;

      track_data_[i].curvature=dTds.dot(track_data_[i].right);
    }
  }

  Track::SampleData Track::generateSampleTrackData(size_t i) const
  {
    SampleData sample;
    size_t centerline_size=centerline_.getPointsSize();
    size_t next=(i+1)%centerline_size;

    const auto& [center_point,dist]=centerline_.getPointAndDist(i);
    btVector3 right_point=
      rightline_.getPoint(rightline_.getClosestIndex(center_point).first);
    btVector3 left_point=
      leftline_.getPoint(leftline_.getClosestIndex(center_point).first);

    sample.position=center_point;
    sample.coverage=dist;

    const auto& next_center_point=centerline_.getPoint(next);
    sample.tangent=next_center_point-center_point;
    sample.tangent.normalize();

    // Create real right vector
    sample.right=right_point-center_point;
    sample.right-=sample.right.dot(sample.tangent)*sample.tangent;
    sample.right.normalize();

    //sample.right=right_point-center_point;
    //sample.right.normalize();

    if(sample.tangent.dot(sample.right)>1e-5)
    {
      std::stringstream ss;
      ss<<"For centerline point i=("<<i;
      ss<<") the dot product of tangent and right vectors is greater than epsilon";
      throw Exception(
          Exception::Type::InternalError,
          ss.str());
    }

    sample.normal=sample.right.cross(sample.tangent);
    sample.normal.normalize();

    if(sample.normal.dot(s_kUp)<0.0)
    {
      std::stringstream ss;
      ss<<"For centerline point i=("<<i;
      ss<<") the dot product of normal and up vectors is negative";
      throw Exception(
          Exception::Type::InternalError,
          ss.str());
    }

    sample.right_width=btVector3(center_point-right_point).length();
    sample.left_width=btVector3(center_point-left_point).length();

    sample.grade=btAsin(sample.tangent.dot(s_kUp));

    btVector3 proj_up=s_kUp-s_kUp.dot(sample.tangent)*sample.tangent;
    btScalar cos_angle=proj_up.dot(sample.normal);
    btScalar sin_angle=proj_up.dot(sample.right);

    sample.bank=btAtan2(sin_angle,cos_angle);

    return sample;
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

    if(!enable_track_data_vec_draw_)
      return;

    const auto& samples=this->getTrackData();

    auto offset=this->getWorldTransform().getOrigin();
    for(const auto& sample:samples)
    {
      auto pos=offset+sample.position;
      p_renderer->drawLine(
          pos,
          pos+sample.normal,
          {0.5,0.5,0.0});
      p_renderer->drawLine(
          pos,
          pos+sample.tangent,
          {0.5,0.5,0.0});
      p_renderer->drawLine(
          pos,
          pos+sample.right*sample.right_width,
          {0.0,0.5,0.0});
      p_renderer->drawLine(
          pos,
          pos+sample.right*-sample.left_width,
          {0.5,0.0,0.0});
    }
  }

  void Track::setWorldTransform(const btTransform& transform)
  {
    transform_=transform;
    for(auto& ground:grounds)
    {
      btTransform new_transform=transform_*ground.getModelTransform();
      ground.setWorldTransform(new_transform);
    }

    // TODO
    // track_data world transform
  }
}
