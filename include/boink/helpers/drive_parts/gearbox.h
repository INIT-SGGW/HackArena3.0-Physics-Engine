#pragma once

#include "boink/helpers/enums.h"

namespace boink
{
struct Gearbox
{
 public:
  static constexpr float kDifferentialRatio = 4.05;
  static constexpr float kGearRatios[9] = {-2.8, 0.0, 2.65, 2.05, 1.72, 1.50, 1.34, 1.21, 1.1};

  Gear current_gear = Gear::First;

  float GetCurrentRatio() const { return kGearRatios[static_cast<uint8_t>(current_gear)]; }
};
}  // namespace boink