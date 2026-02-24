#pragma once

#include <piksel/gui_object.hh>

#include "boink/simulators/weather.h"

namespace boink
{
  class WeatherGui : public piksel::GuiObject
  {
  public:
    WeatherGui(Weather* p_weather);
    void draw() override;
    std::string_view getTitle() const override {return "Weather";}
  private:
    float transition_duration_=0.f;
    Weather* p_weather_;
  };
}
