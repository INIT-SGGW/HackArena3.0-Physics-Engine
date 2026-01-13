#pragma once

#include "boink/helpers/drive_parts/engine.h"
#include "boink/helpers/drive_parts/gearbox.h"
#include "boink/helpers/drive_parts/wheel.h"
#include "boink/helpers/enums.h"

namespace boink
{
struct CarDriveParts
{
  Engine engine;
  Gearbox gearbox;
  Wheel wheels[4];
};
}  // namespace boink