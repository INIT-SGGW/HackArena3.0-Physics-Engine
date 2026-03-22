// clang-format off
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
      version_(Track::extractTrackVersion(filename_)),
      weather_(weather)
  {
    this->initSurfaceInfos();

    GltfExtractor extractor(filename);
    this->initGrounds(extractor);
    this->initFinishLine(extractor);
    this->initPositions(extractor);

    std::string bin_file(filename);
    bin_file+=".boink";

    road_=Road(
        Line::createLine(extractor,CENTERLINE_NAME,finish_line_),
        Line::createLine(extractor,RIGHTLINE_NAME,finish_line_),
        Line::createLine(extractor,LEFTLINE_NAME,finish_line_),
        world_,
        bin_file);
    if(!road_.isClosed())
    {
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Main road is not a closed road");
    }

    pitstop_=Pitstop(extractor,world_,std::string(filename));
    for(const auto& [type,zone]: pitstop_.getZones())
    {
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
      {0.35f, 0.01, 0.3f, wetness,Ground::Type::Grass};
    surface_infos_[Ground::Type::Sand]= 
      {0.3f, 0.25, 0.4f, wetness,Ground::Type::Sand};
    surface_infos_[Ground::Type::Gravel]= 
      {0.45f, 0.15, 0.4f, wetness,Ground::Type::Gravel};
    surface_infos_[Ground::Type::Asphalt]= 
      {1.0f, 0.0f, 0.1f, wetness,Ground::Type::Asphalt};
    surface_infos_[Ground::Type::Kerb]= 
      {1.0f, 0.0f, 0.1f, wetness,Ground::Type::Kerb};
    surface_infos_[Ground::Type::Wall]= 
      {0.0f, 0.0f, 0.1f, wetness, Ground::Type::Wall};
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


  int Track::extractTrackVersion(std::string_view filename)
  {
    size_t i=filename.find("_");
    if(i==std::string_view::npos)
      return -1;

    if(i+1>=filename.length())
      return -1;

    std::string_view version_str=filename.substr(i+1);
    int version;
    auto result=std::from_chars(
        version_str.data(),version_str.data()+version_str.size(),version);

    if(result.ec!=std::errc())
      return -1;

    return version;
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

        drawMetricSample(p_renderer,pos,sample);
      }
      const Line& line=const_cast<const Road&>(road_).getLine(Road::Side::Right);
      for(size_t i=0;i<line.getPointsSize();i++)
      {
        const auto& point=line.getPoint(i);
        size_t i_next=(i+1)%line.getPointsSize();
        p_renderer->drawLine(
          point+offset,
          offset+line.getPoint(i_next),
          {1,1,1});
      }
      const Line& line1=const_cast<const Road&>(road_).getLine(Road::Side::Left);
      for(size_t i=0;i<line1.getPointsSize();i++)
      {
        const auto& point=line1.getPoint(i);
        size_t i_next=(i+1)%line1.getPointsSize();
        p_renderer->drawLine(
          point+offset,
          offset+line1.getPoint(i_next),
          {1,1,1});
      }

    }

    // Pitstop
    {
      for(const auto& [zone_type,_]:pitstop_.getZones())
      {
        btVector3 zone_color;
        switch((Pitstop::Zone)zone_type)
        {
          case Pitstop::Zone::Enter:
            zone_color={0,1,0};
            break;
          case Pitstop::Zone::Fix:
            zone_color={0,0,1};
            break;
          case Pitstop::Zone::Exit:
            zone_color={1,0,0};
            break;
          default:
            zone_color={1,1,1};
            break;
        }
        const auto& zone=pitstop_.getZone((Pitstop::Zone)zone_type);
        const auto& samples=zone.getRoadData();
        auto offset=this->getWorldTransform().getOrigin();
        for(size_t i=0;i<samples.size();i++)
        {
          const auto& sample=samples[i];
          const auto& pos=offset+zone.getPoint(i);;
          drawMetricSample(p_renderer,pos,sample);

          if(i+1!=samples.size())
            p_renderer->drawLine(
              pos,
              offset+zone.getPoint(i+1),
              zone_color);
        }

        const Line& line=const_cast<const Road&>(zone).getLine(Road::Side::Right);
        for(size_t i=0;i<line.getPointsSize()-1;i++)
        {
          const auto& point=line.getPoint(i);
          p_renderer->drawLine(
            point+offset,
            offset+line.getPoint(i+1),
            {1,1,1});
        }
        const Line& line1=const_cast<const Road&>(zone).getLine(Road::Side::Left);
        for(size_t i=0;i<line1.getPointsSize()-1;i++)
        {
          const auto& point=line1.getPoint(i);
          p_renderer->drawLine(
            point+offset,
            offset+line1.getPoint(i+1),
            {1,1,1});
        }
      }
    }

    p_renderer->drawPoint(finish_line_,{1.0,1.0,1.0});
    
    for(const auto& point:start_postions_)
      p_renderer->drawPoint(point,{1.0,1.0,1.0});
  }

  void Track::drawMetricSample(
      Renderer* p_renderer,const btVector3& pos,const Road::Metrics& sample)
  {
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

    btVector3 pos_up=pos+sample.normal*-1.5f;
    p_renderer->drawLine(
        pos_up,
        pos_up+sample.right*sample.right_max_width,
        {1.0,0.5,0.0});
    p_renderer->drawLine(
        pos_up,
        pos_up+sample.right*-sample.left_max_width,
        {0.5,1.0,0.0});
    
    
    // 1. Offset the visualization slightly above ground to avoid Z-fighting
    pos_up = pos + sample.normal * 0.2f; 

    // --- DRAW LEFT SIDE ---
    btVector3 leftCursor = pos_up + (sample.right * -sample.left_width);
    btVector3 leftDir = -sample.right;

    for (const auto& groundSeg : sample.left_grounds) 
    {
        float width = groundSeg.first;
        Ground::Type type = groundSeg.second;

        if (type == Ground::Type::Count) continue;

        btVector3 pStart = leftCursor;
        btVector3 pEnd   = leftCursor + (leftDir * width);

        p_renderer->drawLine(pStart, pEnd, getGroundColor(type));

        // Move the cursor forward for the next segment
        leftCursor = pEnd;
    }

    // --- DRAW RIGHT SIDE ---
    btVector3 rightCursor = pos_up + (sample.right * sample.right_width);
    btVector3 rightDir = sample.right;

    for (const auto& groundSeg : sample.right_grounds) 
    {
        float width = groundSeg.first;
        Ground::Type type = groundSeg.second;

        if (type == Ground::Type::Count) continue;

        btVector3 pStart = rightCursor;
        btVector3 pEnd   = rightCursor + (rightDir * width);

        p_renderer->drawLine(pStart, pEnd, getGroundColor(type));

        // Move the cursor forward for the next segment
        rightCursor = pEnd;
    }
  }

  btVector3 Track::getGroundColor(Ground::Type type) const
  {
    switch (type) {
      case Ground::Type::Asphalt:
      return btVector3(0.15f, 0.15f, 0.15f);
    case Ground::Type::Grass:
      return btVector3(0.13f, 0.55f, 0.13f);
    case Ground::Type::Sand:
      return btVector3(0.76f, 0.70f, 0.20f);
    case Ground::Type::Gravel:
      return btVector3(0.45f, 0.45f, 0.48f);
    case Ground::Type::Wall:
      return btVector3(0.80f, 0.10f, 0.10f);
    case Ground::Type::Kerb:
      return btVector3(0.75f, 0.75f, 0.75f);

    default:
    case Ground::Type::Count:
      return btVector3(1.0f, 0.0f, 1.0f);
    }
  }
}
