#pragma once

namespace boink
{
enum class Gear : int8_t
{
  Reverse = -1,
  Neutral = 0,
  First = 1,
  Second = 2,
  Third = 3,
  Fourth = 4,
  Fifth = 5,
  Sixth = 6,
  Seventh = 7
};

enum class WheelIndex : uint8_t
{
  FL = 0,
  FR = 1,
  RL = 2,
  RR = 3,
};
}  // namespace boink