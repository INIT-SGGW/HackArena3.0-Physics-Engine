#pragma once

#include <LinearMath/btVector3.h>

#include <piksel/gui_object.hh>

namespace boink
{
  class TrackGui : public piksel::GuiObject
  {
  public:
    std::string_view getTitle() const override { return "Track";}
    void draw() override;
  public:
    const btVector3* pos;
    std::string_view filename;
  };
}
