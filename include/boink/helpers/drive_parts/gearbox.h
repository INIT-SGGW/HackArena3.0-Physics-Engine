#pragma once

#include "boink/helpers/enums.h"

namespace boink
{
struct Gearbox
{
 public:
  static constexpr float kDifferentialRatio = 3.9;
  static constexpr float kGearRatios[10] = {-2.8, 0.0, 4.60, 3.40, 2.70, 2.25, 1.90, 1.65, 1.45, 1.30};

  Gear current_gear = Gear::First;

  float GetCurrentRatio() const { return kGearRatios[static_cast<uint8_t>(current_gear)]; }
};
}  // namespace boink