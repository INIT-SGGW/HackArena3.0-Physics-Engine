#pragma once

#include <LinearMath/btScalar.h>

#include <unordered_map>
#include <optional>

namespace boink
{
  struct LapInfo
  {
    static constexpr int kStartingLap=-1;

    int current_lap=kStartingLap;
    btScalar curr_lap_coverage=0.f;
    btScalar curr_lap_time=0.f;

    std::unordered_map<int,btScalar> lap_times_history;

    int getLapsCompleted() const
    {
      int laps_completed=current_lap-kStartingLap-1;

      if(laps_completed<0)
        return 0;
      else
        return laps_completed;
    }

    int getCurrentLap() const
    {
      return current_lap;
    }

    std::optional<std::pair<int,btScalar>> getPersonalBest() const
    {
      bool found=false;
      int lap=-1;
      btScalar best_time=FLT_MAX;
      for(const auto& pair:lap_times_history)
      {
        if(pair.first<=kStartingLap)
          continue;

        if(pair.second<best_time)
        {
          best_time=pair.second;
          lap=pair.first;
          found=true;
        }
      }

      if(found)
        return {{lap,best_time}};
      else
        return std::nullopt;
    }

    std::optional<std::pair<int,btScalar>> getLastLapTime() const
    {
      auto it=lap_times_history.find(current_lap-1);
      if(it==lap_times_history.end())
        return std::nullopt;

      if(it->first<=kStartingLap)
        return std::nullopt;

      return {{it->first,it->second}};
    }


  };
}
