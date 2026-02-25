#pragma once

#include <LinearMath/btVector3.h>
#include <piksel/gui_object.hh>

#include "boink/gui/surface_info_gui.h"

#include <unordered_map>

namespace boink
{
  class Track;
  class TrackGui : public piksel::GuiObject
  {
  public:
    TrackGui(Track* p_track);
    std::string_view getTitle() const override { return "Track";}
    void draw() override;
  private:
    Track* p_track_=nullptr;

    std::unordered_map<Ground::Type,SurfaceInfoGui> surface_guis_;
  };
}
