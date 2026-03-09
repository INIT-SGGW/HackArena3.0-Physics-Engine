#include "boink/gui/weather_gui.h"

#include <imgui.h>

#include "boink/gui/slider_value.h"
#include "boink/simulators/weather.h"

namespace boink
{
  WeatherGui::WeatherGui(Weather* p_weather)
    :p_weather_(p_weather)
  {}

  void WeatherGui::draw()
  {
    SliderValue temperature_celsius_target_=
      SliderValue(p_weather_->temperature_celsius_.getTarget());
    SliderValue cloudiness_target_=
      SliderValue(p_weather_->cloudiness_.getTarget());
    SliderValue rain_indensity_target_=
      SliderValue(p_weather_->rain_indensity_.getTarget());

    ImGui::Text("Weather tuning");
    ImGui::SliderFloat("Sun factor const",&Weather::s_kSunFactorConstant,0,1);
    ImGui::SliderFloat("Temp rate const",&Weather::s_kTempRateConstant,0,1);
    ImGui::SliderFloat("Cloundiness rate const",&Weather::s_kCloundinessRateConstant,0,1);
    ImGui::SliderFloat("Rain add const",&Weather::s_kRainAddConstant,0,1);
    ImGui::SliderFloat("Wetness speed",&Weather::s_kWetnessSpeedConstant,0,10);
    ImGui::NewLine();

    ImGui::SliderFloat(
        "Transition duration [s]",&transition_duration_,0,60);
    ImGui::Text("Temperature: %.2f [C]",
        p_weather_->temperature_celsius_.getCurrent());
    ImGui::SliderFloat("Temperature [C]",&temperature_celsius_target_.get(),1,40);
    ImGui::Text("Cloudiness: %.2f",
        p_weather_->cloudiness_.getCurrent());
    ImGui::SliderFloat("Cloudiness",&cloudiness_target_.get(),0,1);
    ImGui::Text("Rain indensity: %.2f",
        p_weather_->rain_indensity_.getCurrent());
    ImGui::SliderFloat("Rain indensity",&rain_indensity_target_.get(),0,1);

    ImGui::Text("Ground wetness: %.2f",p_weather_->wetness_);

    if(temperature_celsius_target_.hasChanged())
      p_weather_->temperature_celsius_.setTarget(
          temperature_celsius_target_.get(),transition_duration_);
    if(rain_indensity_target_.hasChanged())
      p_weather_->rain_indensity_.setTarget(
          rain_indensity_target_.get(),transition_duration_);
    if(cloudiness_target_.hasChanged())
      p_weather_->cloudiness_.setTarget(
          cloudiness_target_.get(),transition_duration_);
  }
}
