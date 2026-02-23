#pragma once

#include <piksel/gui_object.hh>

#include "boink/smoothed_value.h"

namespace boink
{
  class WeatherGui : public piksel::GuiObject
  {
  public:
    void draw() override;
    std::string_view getTitle() const override {return "Weather";}
  public:
    SmoothedValue* cloudiness=nullptr;
    SmoothedValue* temperature_celsius=nullptr;
    SmoothedValue* rain_indensity=nullptr;
  private:
    float transition_duration_=0.f;
  };
}
