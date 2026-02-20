#pragma once

#include <piksel/gui_object.hh>

namespace boink
{
  class WeatherGui : public piksel::GuiObject
  {
  public:
    void draw() override;
    std::string_view getTitle() const override {return "Weather";}
  public:
    float cloudiness;
    float temperature_celsius;
    float rain_indensity;

    float cloudiness_target;
    float temperature_celsius_target;
    float rain_indensity_target;
  };
}
