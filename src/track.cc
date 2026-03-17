#include "boink/simulators/track/track.h"

#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btScalar.h>
#include <LinearMath/btTransform.h>

#include "boink/exception.h"
#include "boink/gltf_extractor.h"
#include "boink/gui/track_gui.h"
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

    road_=Road(
        Line::createLine(extractor,CENTERLINE_NAME,finish_line_),
        Line::createLine(extractor,RIGHTLINE_NAME,finish_line_),
        Line::createLine(extractor,LEFTLINE_NAME,finish_line_));
    if(!road_.isClosed())
    {
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Main road is not a closed road");
    }

    pitstop_=Pitstop(extractor);
    for(int zone_type=0;zone_type<(int)Pitstop::Zone::Count;zone_type++)
    {
      const auto& zone=pitstop_.getZone((Pitstop::Zone)zone_type);
      if(zone.isClosed())
        throw Exception(
            Exception::Type::UnsupportedFormatError,
            "One of the pitstop roads is closed road");
    }

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

    // Main road
    {
      const auto& samples=road_.getRoadData();

      auto offset=this->getWorldTransform().getOrigin();
      for(size_t i=0;i<samples.size();i++)
      {
        const auto& sample=samples[i];
        const auto& pos=offset+road_.getPoint(i);;
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
    }

    // Pitstop
    {
      for(int zone_type=0;zone_type<(int)Pitstop::Zone::Count;zone_type++)
      {
        const auto& zone=pitstop_.getZone((Pitstop::Zone)zone_type);
        const auto& samples=zone.getRoadData();
        auto offset=this->getWorldTransform().getOrigin();
        for(size_t i=0;i<samples.size();i++)
        {
          const auto& sample=samples[i];
          const auto& pos=offset+zone.getPoint(i);;
          p_renderer->drawPoint(
              pos,
              {0.25,0.25,0.0},
              sample.normal,
              sample.tangent);

          p_renderer->drawLine(
              pos,
              pos+sample.right*sample.right_width,
              {0.0,0.25,0.0});
          p_renderer->drawLine(
              pos,
              pos+sample.right*-sample.left_width,
              {0.25,0.0,0.0});
        }
      }
    }

    p_renderer->drawPoint(finish_line_,{1.0,1.0,1.0});
    
    for(const auto& point:start_postions_)
      p_renderer->drawPoint(point,{1.0,1.0,1.0});
  }
}
