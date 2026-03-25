#pragma once

#include "boink/simulators/vehicle/physics/helpers/enums.h"

namespace boink
{
struct Gearbox
{
 public:
  static constexpr btScalar kDifferentialRatio = 3.9f;
  static constexpr btScalar kGearRatios[10] = {-2.8f, 0.0f, 4.60f, 3.40f, 2.70f, 2.25f, 1.90f, 1.65f, 1.45f, 1.30f};

  Gear current_gear = Gear::First;

  btScalar GetCurrentRatio() const { return kGearRatios[static_cast<uint8_t>(current_gear)]; }
};
}  // namespace boink
