#include "boink/gui/race_gui.h"

#include <imgui.h>

#include "boink/gui/simulation_gui.h"

namespace boink
{
  void RaceGui::draw()
  {
    ImGui::Text("# of vehicles: %lu",num_vehicles);
    SimulationGui::draw();
  }
}
