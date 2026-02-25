#pragma once

#include <piksel/gui_object.hh>

#include "boink/simulators/track/ground.h"

#include <string>

namespace boink
{
  class SurfaceInfoGui : public piksel::GuiObject
  {
  public:
    SurfaceInfoGui(Ground::SurfaceInfo* p_info);
    void draw() override;
    std::string_view getTitle() const override{return title_;}
  private:
    Ground::SurfaceInfo* p_info_=nullptr;
    std::string title_;
  };
}
