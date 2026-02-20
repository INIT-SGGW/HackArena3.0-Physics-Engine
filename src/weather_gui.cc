#include "boink/gui/weather_gui.h"

#include <imgui.h>

namespace boink
{
  void WeatherGui::draw()
  {
    ImGui::SliderFloat("Temperature [C]",&temperature_celsius_target,1,40);
    ImGui::Text("Temperature: %.2f [C]",temperature_celsius);
    ImGui::SliderFloat("Cloudiness",&cloudiness_target,0,1);
    ImGui::Text("Cloudiness: %.2f",cloudiness);
    ImGui::SliderFloat("Rain indensity",&rain_indensity_target,0,1);
    ImGui::Text("Rain indensity: %.2f",rain_indensity);
  }
}
