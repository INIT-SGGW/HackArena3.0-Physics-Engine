#pragma once

#include "boink/simulators/vehicle/physics/helpers/enums.h"

namespace boink
{
struct Gearbox
{
 public:
  static constexpr btScalar kDifferentialRatio = 3.9;
  static constexpr btScalar kGearRatios[10] = {-2.8, 0.0, 4.60, 3.40, 2.70, 2.25, 1.90, 1.65, 1.45, 1.30};

  Gear current_gear = Gear::First;

  btScalar GetCurrentRatio() const { return kGearRatios[static_cast<uint8_t>(current_gear)]; }
};
}  // namespace boink
