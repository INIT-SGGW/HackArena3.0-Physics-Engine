#include "boink/gui/vehicle_gui.h"

#include <imgui.h>

namespace boink
{
  void VehicleGui::draw()
  {
    ImGui::SliderFloat(
        "Friction slip",&friction_slip,0.f,10.f);
    ImGui::SliderFloat(
        "Max suspension force",&max_suspension_force,0.f,50000.f);
    ImGui::SliderFloat(
        "Max suspension travel",&max_suspension_travel_cm,0.f,20.f);
    ImGui::SliderFloat(
        "Suspension compression",&suspension_compression,0.f,100.f);
    ImGui::SliderFloat(
        "Suspension damping",&suspension_damping,0.f,100.f);
    ImGui::SliderFloat(
        "Suspension stifness",&suspension_stiffness,0.f,100.f);
    
    ImGui::Text("Laps completed: %d",laps_completed);
    ImGui::Text("Current lap coverage: %.2f [m]",curr_lap_coverage);
    ImGui::Text("Chassis position: (%.2f,%.2f,%.2f) [m]",
        chassis_position[0],chassis_position[1],chassis_position[2]);
    ImGui::Text("Mass: %.2f [kg]",mass);
    ImGui::Text("Speed: %.2f [m/s]",speed);
    ImGui::Text("COM in CS: (%.2f,%.2f,%.2f) [m]",
        center_of_mass_cs[0],center_of_mass_cs[1],center_of_mass_cs[2]);
  }
}
