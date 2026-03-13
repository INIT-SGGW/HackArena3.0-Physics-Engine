#pragma once

#include <piksel/gui_object.hh>

#include "boink/simulators/simulator.h"
#include "boink/debugger/debugger.h"

#include <vector>
#include <string>

namespace boink
{
  class DebuggerGui : public piksel::GuiObject
  {
  public:
    DebuggerGui(Debugger* p_debug);
    std::string_view getTitle() const override { return "Debug";}
    void draw() override;
  public:
    std::vector<std::pair<Simulator::ID,std::string>> controller_ids;
  private:
    Debugger* p_debug=nullptr;
    bool bullet_draw_=true;
  };
}
