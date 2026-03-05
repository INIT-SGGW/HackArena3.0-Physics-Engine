#pragma once

#include <piksel/gui_object.hh>

#include "boink/simulators/vehicle/ghost_mode.h"

namespace boink
{
  class GhostGui : public piksel::GuiObject
  {
  public:
    GhostGui(GhostMode* p_ghost);
    void draw() override;
    std::string_view getTitle() const override {return "Ghost mode";}
  private:
    GhostMode* p_ghost_=nullptr;
  };
}
