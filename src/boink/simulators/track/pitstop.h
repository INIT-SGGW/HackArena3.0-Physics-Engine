#pragma once

#include "boink/gltf_extractor.h"
#include "boink/simulators/track/road.h"

#include <unordered_map>

namespace boink
{
  class Pitstop
  {
  public:
    enum class Zone
    {
      Enter,
      Fix,
      Exit,
      Count
    };
  public:
    Pitstop()=default;
    Pitstop(const GltfExtractor& extractor);

    const Road& getZone(Zone type) const {return zones_.at(type);}
  public:
    static std::string_view getZoneName(Zone type)
    {return kZonesNames.at(type);}
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
  };
}
