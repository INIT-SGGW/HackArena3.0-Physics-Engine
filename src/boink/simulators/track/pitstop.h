#pragma once

#include "boink/gltf_extractor.h"
#include "boink/simulators/track/road.h"

#include <unordered_map>

namespace boink
{
  class Pitstop
  {
  public:
    enum class Zone : int
    {
      None=1<<0,
      Enter=1<<1,
      Fix=1<<2,
      Exit=1<<3,
    };
  public:
    Pitstop()=default;
    Pitstop(const GltfExtractor& extractor);

    const Road& getZone(Zone type) const;
    const auto& getZones() const {return zones_;}

    btScalar getLength() const {return pitstop_length_;}
  public:
    static std::string_view getZoneName(Zone type)
    {
      if(type==Zone::None)
        return "Unknown";

      return kZonesNames.at(type);
    }
  private:
    static Road createZone(
        const GltfExtractor& extractor,
        const std::string& prefix);
  private:
    static constexpr std::string_view kPitstopNameDelim="_";
    static constexpr std::string_view kPitstopSegName="PITSTOP_ZONE";

    static constexpr std::string_view kLeftSegName="LINE_LEFT";
    static constexpr std::string_view kRightSegName="LINE_RIGHT";
    static constexpr std::string_view kCenterSegName="LINE_CENTER";

    static const std::unordered_map<std::string_view,Zone> kZonesTypes;
    static const std::unordered_map<Zone,std::string_view> kZonesNames;
  private:
    std::unordered_map<Zone,Road> zones_;
    btScalar pitstop_length_=0;
  };
}
