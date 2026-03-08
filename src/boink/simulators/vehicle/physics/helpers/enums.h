#pragma once

#include <ostream>
#include <cstdint>

namespace boink
{
enum class Gear : uint8_t
{
  Reverse = 0,
  Neutral = 1,
  First = 2,
  Second = 3,
  Third = 4,
  Fourth = 5,
  Fifth = 6,
  Sixth = 7,
  Seventh = 8,
  Eighth = 9,
};

// enum class WheelIndex : uint8_t
//{
//   FL = 0,
//   FR = 1,
//   RL = 2,
//   RR = 3,
// };

inline std::ostream& operator<<(std::ostream& os, Gear gear)
{
  switch (gear)
  {
    case Gear::Reverse:
      return os << "Reverse";
    case Gear::Neutral:
      return os << "Neutral";
    case Gear::First:
      return os << "First";
    case Gear::Second:
      return os << "Second";
    case Gear::Third:
      return os << "Third";
    case Gear::Fourth:
      return os << "Fourth";
    case Gear::Fifth:
      return os << "Fifth";
    case Gear::Sixth:
      return os << "Sixth";
    case Gear::Seventh:
      return os << "Seventh";
    case Gear::Eighth:
      return os << "Eighth";
  }
  return os << "Unknown";
}
}  // namespace boink
