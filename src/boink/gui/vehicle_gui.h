#pragma once

#include <piksel/gui_object.hh>

namespace boink
{
  class VehicleGui : public piksel::GuiObject
  {
  public:
    std::string_view getTitle() const override { return "Vehicle";}
    void draw() override;
  public:
    float friction_slip;
    float max_suspension_force;
    float max_suspension_travel_cm;
    float suspension_compression;
    float suspension_damping;
    float suspension_stiffness;

    int laps_completed;
    float curr_lap_coverage;
    float chassis_position[3];

    float mass;
    float speed;
    float center_of_mass_cs[3];
  };
}
