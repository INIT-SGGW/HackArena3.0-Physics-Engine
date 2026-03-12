// clang-format off
#include "boink/debugger/vehicle_controller.h"
#include "boink/utility.h"
#include <GLFW/glfw3.h>

namespace boink
{
VehicleController::VehicleController(
    std::shared_ptr<Vehicle> vehicle,
    const std::unordered_map<Action,int>& key_map)
  : vehicle_(vehicle),key_map_(key_map)
{
  if(map_id_%2==0)
    this->setKeyMap(alternative_key_map_);
  map_id_++;
}

void VehicleController::setVehicle(std::shared_ptr<Vehicle> vehicle) { vehicle_ = vehicle; }

void VehicleController::update(piksel::Window& wnd, piksel::Camera& camera, float)
{
  if (wnd.getKey(key_map_.at(Action::Accelerate)) == piksel::Window::KeyState::Press)
    vehicle_->setEngineForce(1.f);
  else
    vehicle_->setEngineForce(0.f);

  if (wnd.getKey(key_map_.at(Action::Brake)) == piksel::Window::KeyState::Press)
    vehicle_->setBrake(1.f);
  else
    vehicle_->setBrake(0.f);

  if (wnd.getKey(key_map_.at(Action::TurnLeft)) == piksel::Window::KeyState::Press)
    vehicle_->setSteering(0.3f, Vehicle::TurnDirection::Left);
  else if (wnd.getKey(key_map_.at(Action::TurnRight)) == piksel::Window::KeyState::Press)
    vehicle_->setSteering(0.3f, Vehicle::TurnDirection::Right);
  else
    vehicle_->setSteering(0.f, Vehicle::TurnDirection::Right);

  if (wnd.getKey(key_map_.at(Action::GearDown)) == piksel::Window::KeyState::Press)
  {
    if (!is_gear_btn_pressed)
    {
      vehicle_->setGearDown();
      is_gear_btn_pressed = true;
    }
  }

  if (wnd.getKey(key_map_.at(Action::GearUp)) == piksel::Window::KeyState::Press)
  {
    if (!is_gear_btn_pressed)
    {
      vehicle_->setGearUp();
      is_gear_btn_pressed = true;
    }
  }

  if (wnd.getKey(key_map_.at(Action::GearUp)) == piksel::Window::KeyState::Release &&
      wnd.getKey(key_map_.at(Action::GearDown)) == piksel::Window::KeyState::Release)
    is_gear_btn_pressed = false;

  const auto& trans = vehicle_->getChassisWorldTransform();
  btVector3 back = trans.getBasis() * btVector3(0.f, 0.25f, -1.f);
  btVector3 cam_pos = trans.getOrigin() + back * 15;

  camera.set(math::bt2glm(trans.getOrigin()), math::bt2glm(cam_pos));
}

void VehicleController::setKeyMap(std::unordered_map<Action,int> key_map)
{
  key_map_=std::move(key_map);
}

const std::unordered_map<VehicleController::Action,int>
VehicleController::default_key_map_={
  {Action::Accelerate,GLFW_KEY_UP},
  {Action::Brake,GLFW_KEY_DOWN},
  {Action::TurnLeft,GLFW_KEY_LEFT},
  {Action::TurnRight,GLFW_KEY_RIGHT},
  {Action::GearUp,GLFW_KEY_X},
  {Action::GearDown,GLFW_KEY_Z},
};

const std::unordered_map<VehicleController::Action,int>
VehicleController::alternative_key_map_={
  {Action::Accelerate,GLFW_KEY_W},
  {Action::Brake,GLFW_KEY_S},
  {Action::TurnLeft,GLFW_KEY_A},
  {Action::TurnRight,GLFW_KEY_D},
  {Action::GearUp,GLFW_KEY_LEFT_SHIFT},
  {Action::GearDown,GLFW_KEY_LEFT_CONTROL},
};

int VehicleController::map_id_=0;
}  // namespace boink
