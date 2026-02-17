#include "boink/gui/simulation_gui.h"

#include <imgui.h>
#include <piksel/gui_object.hh>

#include <algorithm>
#include <string>

namespace boink
{
  void SimulationGui::draw()
  {
    ImGui::Checkbox("Freeze simulation",&freeze);
    ImGui::Text("Simulation duration: %.2f [s]",duration);
    ImGui::Text("Gravitational acceleration: %.2f [m/s^2]",gravity_acc);
    ImGui::Text("# of simulators: %lu",num_simulators);

    for(const auto& p : guis_)
    {
      std::string label(p.second->getTitle());
      label+=" id=";
      label+=std::to_string(p.first);

      ImGui::PushID((int)p.first);
      if(ImGui::CollapsingHeader(label.c_str()))
        p.second->draw();
      ImGui::PopID();
    }
  }

  void SimulationGui::addSimulatorGui(
      Simulator::ID id,std::shared_ptr<piksel::GuiObject> gui)
  {
    guis_.emplace_back(id,gui);
  }

  void SimulationGui::removeSimulatorGui(Simulator::ID id)
  {
    auto it=std::find_if(guis_.begin(),guis_.end(),
        [=](const auto& p) {return p.first==id;});

    guis_.erase(it);
  }
 }
