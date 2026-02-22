#pragma once

#include <piksel/gui_object.hh>

#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

namespace boink
{
  class VehicleGui : public piksel::GuiObject
  {
  public:
    std::string_view getTitle() const override { return "Vehicle";}
    void draw() override;
  public:
    btRaycastVehicle::btVehicleTuning* tunning=nullptr;

    int laps_completed;
    float curr_lap_coverage;
    float chassis_position[3];

    float mass;
    float speed;
    float center_of_mass_cs[3];
  };
}
