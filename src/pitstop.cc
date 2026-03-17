#include "boink/simulators/track/pitstop.h"

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

      BOINK_TRACE("Zone type: {}",kZonesNames.at(type));
      for(const auto& point:getZone(type).getLine(Road::Side::Center).getPointsAndDist())
      {
        BOINK_TRACE("Vec: {} dist={}",point.first,point.second);
      }
    }

  }

  Road Pitstop::createZone(
      const GltfExtractor& extractor,
      const std::string& prefix)
  {
    std::stringstream ss;

    ss<<prefix<<kPitstopNameDelim<<kCenterSegName;
    Line center=Line::createLine(extractor,ss.str());
    ss.str("");

    ss<<prefix<<kPitstopNameDelim<<kLeftSegName;
    Line left=Line::createLine(extractor,ss.str());
    ss.str("");

    ss<<prefix<<kPitstopNameDelim<<kRightSegName;
    Line right=Line::createLine(extractor,ss.str());
    ss.str("");

    return Road(center,right,left);
  }

  const std::unordered_map<std::string_view,Pitstop::Zone> Pitstop::kZonesTypes=
  {
    {"ENTER",Zone::Enter},
    {"FIX",Zone::Fix},
    {"EXIT",Zone::Exit}
  };

  const std::unordered_map<Pitstop::Zone,std::string_view> Pitstop::kZonesNames=
  {
    {Zone::Enter,"ENTER"},
    {Zone::Fix,"FIX"},
    {Zone::Exit,"EXIT"}
  };
}
