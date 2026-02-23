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
    const float* cloudiness;
    const float* temperature_celsius;
    const float* rain_indensity;

    float* cloudiness_target;
    float* temperature_celsius_target;
    float* rain_indensity_target;
  };
}
