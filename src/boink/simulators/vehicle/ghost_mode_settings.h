#pragma once

#include <LinearMath/btScalar.h>

namespace boink
{
  struct GhostModeSettings
  {
    btScalar enter_delay;
    btScalar max_enter_speed;

    btScalar exit_delay;
    btScalar min_exist_speed;

    unsigned int enabled_until_completed_laps;

    btScalar exit_delay_when_overlap;
  };
}
