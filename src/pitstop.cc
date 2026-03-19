#include "boink/simulators/track/pitstop.h"

#include "boink/exception.h"
#include "boink/logger.h"
#include "boink/assert.h"

#include <sstream>
#include <unordered_map>

namespace boink
{
  Pitstop::Pitstop(const GltfExtractor& extractor)
    :pitstop_length_(0)
  {
    for(auto& [type,name]:kZonesNames)
    {
      std::stringstream ss;
      ss<<kPitstopSegName<<kPitstopNameDelim<<name;
      zones_[type]=createZone(extractor,ss.str());

      if(zones_[type].isClosed())
        throw Exception(
            Exception::Type::UnsupportedFormatError,
            "Pitstop line retured is not open");

      BOINK_TRACE("Zone type: {} {}",kZonesNames.at(type),"center");
      for(const auto& point:getZone(type).getLine(Road::Side::Center).getPointsAndDist())
        BOINK_TRACE("Vec: {} dist={}",point.first,point.second);

      BOINK_TRACE("Zone type: {} {}",kZonesNames.at(type),"left");
      for(const auto& point:getZone(type).getLine(Road::Side::Left).getPointsAndDist())
        BOINK_TRACE("Vec: {} dist={}",point.first,point.second);

      BOINK_TRACE("Zone type: {} {}",kZonesNames.at(type),"right");
      for(const auto& point:getZone(type).getLine(Road::Side::Right).getPointsAndDist())
        BOINK_TRACE("Vec: {} dist={}",point.first,point.second);

      pitstop_length_+=zones_[type].getLength();
    }
  }

  const Road& Pitstop::getZone(Pitstop::Zone type) const 
  {
    auto it=zones_.find(type);
    if(it==zones_.end())
    {
      BOINK_ASSERT(false,"type was Zone::Node");

      return zones_.begin()->second;
    }
    return it->second;
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

    return Road(std::move(center),std::move(right),std::move(left));
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
