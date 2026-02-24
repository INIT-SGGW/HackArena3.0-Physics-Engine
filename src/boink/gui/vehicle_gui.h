#pragma once

#include <piksel/gui_object.hh>

#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

namespace boink
{
  class Vehicle;
  class VehicleGui : public piksel::GuiObject
  {
  public:
    VehicleGui(Vehicle* p_vehicle);
    std::string_view getTitle() const override { return "Vehicle";}
    void draw() override;
  public:
    Vehicle* p_vehicle_=nullptr;
  };
}
