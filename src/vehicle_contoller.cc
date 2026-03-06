#include "boink/debugger/vehicle_controller.h"
#include "boink/utility.h"

namespace boink
{
VehicleController::VehicleController(std::shared_ptr<Vehicle> vehicle) : vehicle_(vehicle) {}

void VehicleController::setVehicle(std::shared_ptr<Vehicle> vehicle) { vehicle_ = vehicle; }

void VehicleController::update(piksel::Window& wnd, piksel::Camera& camera, float)
{
  if (wnd.getKey(GLFW_KEY_UP) == piksel::Window::KeyState::Press)
    vehicle_->setEngineForce(1.f);
  else
    vehicle_->setEngineForce(0.f);

  if (wnd.getKey(GLFW_KEY_SPACE) == piksel::Window::KeyState::Press)
    vehicle_->setBrake(1.f);
  else
    vehicle_->setBrake(0.f);

  if (wnd.getKey(GLFW_KEY_LEFT) == piksel::Window::KeyState::Press)
    vehicle_->setSteering(0.3f, Vehicle::TurnDirection::Left);
  else if (wnd.getKey(GLFW_KEY_RIGHT) == piksel::Window::KeyState::Press)
    vehicle_->setSteering(0.3f, Vehicle::TurnDirection::Right);
  else
    vehicle_->setSteering(0.f, Vehicle::TurnDirection::Right);

  if (wnd.getKey(GLFW_KEY_Z) == piksel::Window::KeyState::Press)
  {
    if (!is_gear_btn_pressed)
    {
      vehicle_->setGearDown();
      is_gear_btn_pressed = true;
    }
  }

  if (wnd.getKey(GLFW_KEY_X) == piksel::Window::KeyState::Press)
  {
    if (!is_gear_btn_pressed)
    {
      vehicle_->setGearUp();
      is_gear_btn_pressed = true;
    }
  }

  if (wnd.getKey(GLFW_KEY_X) == piksel::Window::KeyState::Release &&
      wnd.getKey(GLFW_KEY_Z) == piksel::Window::KeyState::Release)
    is_gear_btn_pressed = false;

  const auto& trans = vehicle_->getChassisWorldTransform();
  btVector3 back = trans.getBasis() * btVector3(0.f, 0.25f, -1.f);
  btVector3 cam_pos = trans.getOrigin() + back * 15;

  camera.set(bt2glm(trans.getOrigin()), bt2glm(cam_pos));
}
}  // namespace boink
