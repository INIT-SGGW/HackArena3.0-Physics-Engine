#pragma once

#include <piksel/gui_object.hh>

#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

#include "boink/simulators/vehicle/wheel_position.h"

namespace boink
{
  class Vehicle;
  class VehicleGui : public piksel::GuiObject
  {
  public:
    VehicleGui(Vehicle* p_vehicle);
    std::string_view getTitle() const override { return "Vehicle";}
    void draw() override;
  private:
    void drawWheel(WheelPosition pos);
  public:
    bool mesh_enabled=true;
    bool collider_enabled=true;
  private:
    Vehicle* p_vehicle_=nullptr;
  };
}
