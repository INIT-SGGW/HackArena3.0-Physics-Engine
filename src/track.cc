#include "boink/simulators/track/track.h"

#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btScalar.h>
#include <LinearMath/btTransform.h>

#include "boink/exception.h"
#include "boink/gltf_extractor.h"
#include "boink/gui/track_gui.h"
#include "boink/constants.h"
#include "boink/assert.h"

#include <algorithm>
#include <cctype>
#include <memory>
#include <optional>
#include <sstream>
#include <system_error>
#include <vector>
#include <iostream>
#include <sstream>
#include <charconv>
#include <random>

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
    this->initFinishLine(extractor);
    this->initPositions(extractor);
    this->createLines(extractor);

    this->createTrackData();
    gui_=std::make_shared<TrackGui>(this);
  }

  std::shared_ptr<piksel::GuiObject> Track::getGui() 
  {
    return gui_;
  }

  btVector3 Track::getStartingPosition(size_t position) const
  {
    size_t index=position-1;

    BOINK_ASSERT(index<start_postions_.size());

    return start_postions_.at(index);
  }

  btVector3 Track::getOnTrackRandomPosition() const
  {
    std::random_device rd;
    static std::mt19937 gen(rd());

    std::uniform_int_distribution<size_t> dist(0,centerline_.getPointsSize()-1);
    size_t random_index=dist(gen);

    BOINK_ASSERT(random_index<track_data_.size());
    if(random_index>=track_data_.size())
      return centerline_.getPoint(random_index);

    const auto& sample=track_data_.at(random_index);
    BOINK_ASSERT(
        (sample.position-centerline_.getPoint(random_index)).length2()<g_Epsilon);

    std::uniform_real_distribution<btScalar> real_dist(0.0f,1.f);
    btScalar random_left_width=real_dist(gen)*sample.left_width;
    btScalar random_right_width=real_dist(gen)*sample.right_width;

    btVector3 offset=sample.right*(random_right_width-random_left_width);
    btVector3 random_point=centerline_.getPoint(random_index);

    return random_point+offset;
  }

  Track::SampleData Track::getClosestTrackSample(const btVector3& point) const
  {
    size_t index=std::distance(centerline_.begin(), centerline_.getClosest(point));
    
    BOINK_ASSERT(index<track_data_.size());
    if(index>=track_data_.size())
      return track_data_.at(0);;

    const auto& sample=track_data_.at(index);
    BOINK_ASSERT(
        (sample.position-centerline_.getPoint(index)).length2()<g_Epsilon);

    return sample;
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
    surface_infos_[Ground::Type::Wall]= 
      {1.0f,0.1f,wetness,Ground::Type::Wall};
  }

  void Track::initGrounds(const GltfExtractor& extractor)
  {
    const auto& nodes=extractor.getNodes();

    for(const auto& node : nodes)
    {
      if(node.vertices.size()==0 || node.indices.size()==0)
        continue;

      std::optional<Ground::Type> type=Track::resolveGroundTypeFromName(node.name);

      if(!type.has_value())
        continue;
      if(node.type!=TINYGLTF_MODE_TRIANGLES)
        throw Exception(
            Exception::Type::UnsupportedFormatError,
            "Node type is not TRIANGLES_MODE");

      grounds.emplace_back(
          node.vertices,
          node.indices,
          node.transform,
          &surface_infos_[type.value()],world_);
    }
  }

  void Track::createLines(const GltfExtractor& extractor)
  {
    centerline_=Line::createLine(extractor,finish_line_,CENTERLINE_NAME);
    rightline_=Line::createLine(extractor,finish_line_,RIGHTLINE_NAME);
    leftline_=Line::createLine(extractor,finish_line_,LEFTLINE_NAME);

    if(
        !centerline_.isClosed() || 
        !rightline_.isClosed() || 
        !leftline_.isClosed())
    {
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Center, right or left line is not closed line");
    }


    // Check if centerline should be reveresed
    {
      auto center_point=centerline_.getPoint(0);
      auto next_center_point=centerline_.getPoint(1);

      auto dir=next_center_point-center_point;

      auto right_point=rightline_.getClosest(center_point)->first;
      auto right=right_point-center_point;

      auto normal=right.cross(dir);

      if(normal.dot(g_Up)<0)
        centerline_.reverse();
    }

  }

  void Track::initPositions(const GltfExtractor& extractor)
  {
    const auto& nodes=extractor.getNodes();

    std::vector<std::pair<size_t,btVector3>> postion_pairs;

    for(const auto& node : nodes)
    {
      size_t pos=node.name.find(POSITION_SEG_NAME);
      if(pos==std::string::npos)
        continue;

      pos+=DELIM.size()+POSITION_SEG_NAME.size();

      std::string pos_num_str=node.name.substr(pos);
      unsigned int pos_num;
      
      auto [ptr, ec]=
        std::from_chars(
            pos_num_str.data(),pos_num_str.data()+pos_num_str.size(),pos_num);

      if(!(ec==std::errc()&&(pos_num_str.data()+pos_num_str.size())==ptr))
        throw Exception(
            Exception::Type::UnsupportedFormatError,
            "Cannot extract postion number from node name");

      if(pos_num==0)
        throw Exception(
            Exception::Type::UnsupportedFormatError,
            "Position num cannot be zero");

      postion_pairs.push_back({pos_num,node.transform.getOrigin()});
    }

    start_postions_.resize(postion_pairs.size());
    std::vector<bool> pos_exist(postion_pairs.size(),false);

    // verify if there are all postions
    for(const auto& pair:postion_pairs)
    {
      if(pair.first-1>=pos_exist.size())
        throw Exception(
            Exception::Type::UnsupportedFormatError,
            "Cannot be position greater from number of positions");

      if(pos_exist[pair.first-1]==true)
        throw Exception(
            Exception::Type::UnsupportedFormatError,
            "Found duplicated position");

      pos_exist[pair.first-1]=true;
      start_postions_[pair.first-1]=pair.second;
    }

    if(auto it=std::find(pos_exist.begin(),pos_exist.end(),false);it!=pos_exist.end())
    {
      std::stringstream ss;
      ss<<"Position: "<<it-pos_exist.begin()+1;
      throw Exception(
        Exception::Type::UnsupportedFormatError,
        ss.str());
    }
  }

  void Track::initFinishLine(const GltfExtractor& extractor)
  {
    const auto& node= extractor.getNode(FINISH_LANE_NAME);

    finish_line_=node.transform.getOrigin();
  }

  void Track::createTrackData()
  {
    track_data_.reserve(centerline_.getPointsSize());

    BOINK_ASSERT(centerline_.getPointsSize()>2);
    BOINK_ASSERT(rightline_.getPointsSize()>2);
    BOINK_ASSERT(leftline_.getPointsSize()>2);

    for(size_t i=0;i<centerline_.getPointsSize();i++)
      track_data_.push_back(this->generateSampleTrackData(i));

    // Calc curvature 
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

    if(track_data_.size()!=centerline_.getPointsSize())
      throw Exception(
          Exception::Type::InternalError,
          "After read track_data and center line points sizes does not match");
  }

  Track::SampleData Track::generateSampleTrackData(size_t i) const
  {
    SampleData sample;
    size_t centerline_size=centerline_.getPointsSize();
    size_t next=(i+1)%centerline_size;

    const auto& [center_point,dist]=centerline_.getPointAndDist(i);
    btVector3 right_point=
      rightline_.getClosest(center_point)->first;

    sample.position=center_point;
    sample.coverage=dist;

    const auto& next_center_point=centerline_.getPoint(next);
    sample.tangent=next_center_point-center_point;
    sample.tangent.normalize();

    // Create real right vector
    sample.right=right_point-center_point;
    sample.right-=sample.right.dot(sample.tangent)*sample.tangent;
    sample.right.normalize();

    if(sample.tangent.dot(sample.right)>g_Epsilon)
    {
      std::stringstream ss;
      ss<<"For centerline point i=("<<i;
      ss<<") the dot product of tangent and right vectors is greater than epsilon";
      throw Exception(
          Exception::Type::InternalError,
          ss.str());
    }

    sample.normal=sample.right.cross(sample.tangent);
    BOINK_ASSERT(sample.normal.length2()>g_Epsilon);
    sample.normal.normalize();

    if(sample.normal.dot(g_Up)<0.0)
    {
      std::stringstream ss;
      ss<<"For centerline point i=("<<i;
      ss<<") the dot product of normal and up vectors is negative";
      throw Exception(
          Exception::Type::InternalError,
          ss.str());
    }

    sample.right_width=(rightline_.getClosestPointInterpolated(
        center_point).first-center_point).length();
    sample.left_width=(leftline_.getClosestPointInterpolated(
        center_point).first-center_point).length();

    sample.grade=btAsin(sample.tangent.dot(g_Up));

    btVector3 proj_up=g_Up-g_Up.dot(sample.tangent)*sample.tangent;
    btScalar cos_angle=proj_up.dot(sample.normal);
    btScalar sin_angle=proj_up.dot(sample.right);

    sample.bank=btAtan2(sin_angle,cos_angle);

    return sample;
  }

  std::optional<Ground::Type> Track::resolveGroundTypeFromName(std::string name)
  {
    std::vector<std::string> ground_names;
    ground_names.reserve((size_t)Ground::Type::Count);
    for(int i=0;i<(int)Ground::Type::Count;i++)
      ground_names.emplace_back(Ground::toString((Ground::Type)i));

    std::transform(name.begin(),name.end(),name.begin(),
        [](auto c)
        {
          return std::tolower(c);
        });

    for(size_t i=0;i<ground_names.size();i++)
    {
      size_t count=name.find(ground_names[i]);
      if(count!=std::string::npos)
        return (Ground::Type)i;
    }

    return std::nullopt;
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
      p_renderer->drawPoint(
          pos,
          {0.5,0.5,0.0},
          sample.normal,
          sample.tangent);

      p_renderer->drawLine(
          pos,
          pos+sample.right*sample.right_width,
          {0.0,0.5,0.0});
      p_renderer->drawLine(
          pos,
          pos+sample.right*-sample.left_width,
          {0.5,0.0,0.0});
    }

    p_renderer->drawPoint(finish_line_,{1.0,1.0,1.0});
    
    for(const auto& point:start_postions_)
      p_renderer->drawPoint(point,{1.0,1.0,1.0});
  }
}
