#pragma once

#include <LinearMath/btScalar.h>

#include <unordered_map>

namespace boink
{
  struct LapInfo
  {
    static constexpr int kStartingLap=-1;
    int laps_completed=kStartingLap;
    btScalar curr_lap_coverage=0.f;
    btScalar curr_lap_time=0.f;

    std::unordered_map<int,btScalar> lap_times_history;

    std::pair<int,btScalar> getPersonalBest() const
    {
      int lap;
      btScalar best_time=FLT_MAX;
      for(const auto& pair:lap_times_history)
      {
        if(pair.first<=kStartingLap)
          continue;

        if(pair.second<best_time)
        {
          best_time=pair.second;
          lap=pair.first;
        }
      }

      return {lap,best_time};
    }
  };
}
