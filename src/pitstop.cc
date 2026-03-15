#include "boink/simulators/track/pitstop.h"

#include "boink/constants.h"
#include "boink/logger.h"

#include <sstream>
#include <unordered_map>

namespace boink
{
  Pitstop::Pitstop(const GltfExtractor& extractor)
  {
    for(auto& [type,name]:kZonesNames)
    {
      std::stringstream ss;
      ss<<kPitstopSegName<<kPitstopNameDelim<<name;
      zones_[type]=createZone(extractor,ss.str());

      if(!isInOrder(zones_[type].center,zones_[type].right))
        zones_[type].center.reverse();
    }

    for(const auto& [type,zone]:zones_)
    {
      BOINK_TRACE("Zone type: {}",kZonesNames.at(type));
      for(const auto& point:zone.center)
      {
        BOINK_TRACE("Vec: {} dist={}",point.first,point.second);
      }
    }
  }

  // Function checks whether center line points are increasing in the forward
  // direction
  bool Pitstop::isInOrder(const Line& center,const Line& right)
  {
    auto center_point=center.getPoint(0);
    auto next_center_point=center.getPoint(1);

    auto dir=next_center_point-center_point;

    auto right_point=right.getClosest(center_point)->first;
    auto right_dir=right_point-center_point;

    auto normal=right_dir.cross(dir);

    return normal.dot(g_Up)>0;
  }

  Pitstop::Zone Pitstop::createZone(
      const GltfExtractor& extractor,
      const std::string& prefix)
  {
    std::stringstream ss;
    Zone zone;

    ss<<prefix<<kPitstopNameDelim<<kCenterSegName;
    zone.center=Line::createLine(extractor,ss.str());
    ss.str("");

    ss<<prefix<<kPitstopNameDelim<<kLeftSegName;
    zone.left=Line::createLine(extractor,ss.str());
    ss.str("");

    ss<<prefix<<kPitstopNameDelim<<kRightSegName;
    zone.right=Line::createLine(extractor,ss.str());
    ss.str("");

    return zone;
  }

  const std::unordered_map<std::string_view,Pitstop::ZoneType> Pitstop::kZonesTypes=
  {
    {"ENTER",ZoneType::Enter},
    {"FIX",ZoneType::Fix},
    {"EXIT",ZoneType::Exit}
  };

  const std::unordered_map<Pitstop::ZoneType,std::string_view> Pitstop::kZonesNames=
  {
    {ZoneType::Enter,"ENTER"},
    {ZoneType::Fix,"FIX"},
    {ZoneType::Exit,"EXIT"}
  };
}
