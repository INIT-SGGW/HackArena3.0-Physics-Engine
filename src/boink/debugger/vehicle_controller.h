#pragma once

#include <piksel/camera.hh>
#include <piksel/window.hh>

#include "boink/debugger/controller.h"
#include "boink/simulators/vehicle/vehicle.h"

#include <memory>

namespace boink
{
  class VehicleController : public Controller
  {
  public:
    VehicleController(std::shared_ptr<Vehicle> vehicle);
    void setVehicle(std::shared_ptr<Vehicle> vehicle);
    void update(piksel::Window& wnd, piksel::Camera& camera,float dt) override;
  private:
    std::shared_ptr<Vehicle> vehicle_;
  };
}
