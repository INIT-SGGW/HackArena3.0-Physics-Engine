#include "boink/debugger/camera_controller.h"
#include <piksel/window.hh>

namespace boink
{
  void CameraController::update(
      piksel::Window& wnd, 
      piksel::Camera& cam,
      float dt)
  {
    piksel::Window::MousePos mouse_pos=wnd.getMousePos();

    if(wnd.getKey(GLFW_KEY_W)==piksel::Window::KeyState::Press)
      cam.moveLongitudinal(dt);
    if(wnd.getKey(GLFW_KEY_S)==piksel::Window::KeyState::Press)
      cam.moveLongitudinal(-dt);

    if(wnd.getKey(GLFW_KEY_A)==piksel::Window::KeyState::Press)
      cam.moveLateral(-dt);
    if(wnd.getKey(GLFW_KEY_D)==piksel::Window::KeyState::Press)
      cam.moveLateral(dt);

    wnd.setCursor(false);
    cam.rotateYaw((float)(prev_mouse_pos_.x-mouse_pos.x)*dt);
    cam.rotatePitch((float)(prev_mouse_pos_.y-mouse_pos.y)*dt);

    prev_mouse_pos_.x=mouse_pos.x;
    prev_mouse_pos_.y=mouse_pos.y;
  }

  void CameraController::updateMouse(piksel::Window& wnd)
  {
    prev_mouse_pos_=wnd.getMousePos();
  }
}
