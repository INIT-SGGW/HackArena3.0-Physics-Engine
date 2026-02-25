#pragma once

#include <piksel/gui_object.hh>
#include "boink/simulators/simulator.h"
#include <vector>
#include <string>

namespace boink
{
  class DebuggerGui : public piksel::GuiObject
  {
  public:
    std::string_view getTitle() const override { return "Debug";}
    void draw() override;
  public:
    float* mouse_speed=nullptr;
    float* camera_speed=nullptr;
    const float* fps=nullptr;
    std::vector<std::pair<Simulator::ID,std::string>> controller_ids;
    int* selected_controller=nullptr;
  };
}
