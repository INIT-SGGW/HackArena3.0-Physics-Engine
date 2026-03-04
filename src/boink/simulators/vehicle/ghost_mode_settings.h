#pragma once

#include <LinearMath/btScalar.h>

namespace boink
{
  struct GhostModeSettings
  {
    btScalar enter_delay=0.f;
    btScalar max_enter_speed=0.f;

    btScalar exit_delay=0.f;
    btScalar min_exist_speed=0.f;

    unsigned int enabled_until_completed_laps=0;

    btScalar exit_delay_when_overlap=0.f;
  };
}
