#pragma once

#include <memory>
#include <piksel/camera.hh>
#include <piksel/window.hh>
#include <unordered_map>

#include "boink/debugger/controller.h"
#include "boink/simulators/vehicle/vehicle.h"

namespace boink
{
class VehicleController : public Controller
{
public:
  enum class Action
  {
    Accelerate,
    Brake,
    TurnLeft,
    TurnRight,
    GearUp,
    GearDown
  };
public:
  VehicleController(
      std::shared_ptr<Vehicle> vehicle, 
      const std::unordered_map<Action,int>& key_map=default_key_map_);
  void setVehicle(std::shared_ptr<Vehicle> vehicle);
  void update(piksel::Window& wnd, piksel::Camera& camera, float dt) override;
  void setKeyMap(std::unordered_map<Action,int> key_map);
public:
  static const std::unordered_map<Action,int> default_key_map_;
  static const std::unordered_map<Action,int> alternative_key_map_;
  static int map_id_;
private:
  std::shared_ptr<Vehicle> vehicle_;
  bool is_gear_btn_pressed = false;
  std::unordered_map<Action,int> key_map_;
};
}  // namespace boink
