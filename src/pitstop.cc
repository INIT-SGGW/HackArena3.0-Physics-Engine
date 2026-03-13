#include "boink/simulators/track/pitstop.h"

#include "boink/constants.h"

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
  }

  // Function checks whether center line points are increasing in the forward
  // direction
  bool Pitstop::isInOrder(const Line& center,const Line& right)
  {
    auto center_point=center.getPoint(0);
    auto next_center_point=center.getPoint(1);

    auto dir=next_center_point-center_point;

    auto right_point=right.getPoint( 
        right.getClosestIndex(center_point).first);
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
