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
    Pitstop(const GltfExtractor& extractor);

  private:
    static bool isInOrder(const Line& center,const Line& right);
    static Zone createZone(
        const GltfExtractor& extractor,
        const std::string& prefix);
  private:
    static constexpr std::string_view kPitstopNameDelim="_";
    static constexpr std::string_view kPitstopSegName="PITSTOP_ZONE";

    static constexpr std::string_view kLeftSegName="LEFT";
    static constexpr std::string_view kRightSegName="RIGHT";
    static constexpr std::string_view kCenterSegName="CENTER";

    static const std::unordered_map<std::string_view,ZoneType> kZonesTypes;
    static const std::unordered_map<ZoneType,std::string_view> kZonesNames;
  private:
    std::unordered_map<ZoneType,Zone> zones_;
  };
}
