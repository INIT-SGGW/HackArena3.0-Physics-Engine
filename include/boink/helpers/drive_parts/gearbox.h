#pragma once

#include "boink/helpers/enums.h"

namespace boink
{
struct Gearbox
{
 public:
  static constexpr float kDifferentialRatio = 4.05;

  Gear current_gear = Gear::Neutral;

  float GetCurrentRatio() const
  {
    if (current_gear == Gear::Reverse)
    {
      return kReverseGearRatio;
    }
    else if (current_gear == Gear::Neutral)
    {
      return 0.0f;
    }
    else
    {
      return kGearRatios[static_cast<int>(current_gear) - 1];
    }
  }

 private:
  static constexpr float kGearRatios[7] = {2.65, 2.05, 1.72, 1.50, 1.34, 1.21, 1.1};
  static constexpr float kReverseGearRatio = 2.8;
};
}  // namespace boink