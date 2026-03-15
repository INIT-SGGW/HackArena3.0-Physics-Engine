#pragma once

#include "boink/gltf_extractor.h"
#include "boink/simulators/track/line.h"

#include <unordered_map>

namespace boink
{
  class Pitstop
  {
  public:
    enum class ZoneType
    {
      Enter,
      Fix,
      Exit,
      Count
    };
    struct Zone
    {
      Line right;
      Line center;
      Line left;
    };
  public:
    Pitstop()=default;
    Pitstop(const GltfExtractor& extractor);

    const Zone getZone(ZoneType type) const {return zones_.at(type);}
  private:
    static bool isInOrder(const Line& center,const Line& right);
    static Zone createZone(
        const GltfExtractor& extractor,
        const std::string& prefix);
  private:
    static constexpr std::string_view kPitstopNameDelim="_";
    static constexpr std::string_view kPitstopSegName="PITSTOP_ZONE";

    static constexpr std::string_view kLeftSegName="LINE_LEFT";
    static constexpr std::string_view kRightSegName="LINE_RIGHT";
    static constexpr std::string_view kCenterSegName="LINE_CENTER";

    static const std::unordered_map<std::string_view,ZoneType> kZonesTypes;
    static const std::unordered_map<ZoneType,std::string_view> kZonesNames;
  private:
    std::unordered_map<ZoneType,Zone> zones_;
  };
}
