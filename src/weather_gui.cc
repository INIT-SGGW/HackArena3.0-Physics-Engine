#include "boink/gui/weather_gui.h"

#include <imgui.h>

#include "boink/gui/slider_value.h"

namespace boink
{
  void WeatherGui::draw()
  {
    SliderValue temperature_celsius_target_=
      SliderValue(temperature_celsius->getTarget());
    SliderValue cloudiness_target_=
      SliderValue(cloudiness->getTarget());
    SliderValue rain_indensity_target_=
      SliderValue(rain_indensity->getTarget());

    ImGui::SliderFloat(
        "Transition duration [s]",&transition_duration_,0,60);
    ImGui::Text("Temperature: %.2f [C]",temperature_celsius->getCurrent());
    ImGui::SliderFloat("Temperature [C]",&temperature_celsius_target_.get(),1,40);
    ImGui::Text("Cloudiness: %.2f",cloudiness->getCurrent());
    ImGui::SliderFloat("Cloudiness",&cloudiness_target_.get(),0,1);
    ImGui::Text("Rain indensity: %.2f",rain_indensity->getCurrent());
    ImGui::SliderFloat("Rain indensity",&rain_indensity_target_.get(),0,1);

    if(temperature_celsius_target_.hasChanged())
      temperature_celsius->setTarget(
          temperature_celsius_target_.get(),transition_duration_);
    if(rain_indensity_target_.hasChanged())
      rain_indensity->setTarget(
          rain_indensity_target_.get(),transition_duration_);
    if(cloudiness_target_.hasChanged())
      cloudiness->setTarget(
          cloudiness_target_.get(),transition_duration_);
  }
}
