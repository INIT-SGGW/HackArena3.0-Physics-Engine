#pragma once

#include "boink/gui/simulation_gui.h"

namespace boink
{
  class RaceGui : public SimulationGui
  {
  public:
    std::string_view getTitle() const override {return "Race";}
    void draw() override;
  public:
    bool enable_ghost_sim_=false;
    size_t num_vehicles;
  };
}
