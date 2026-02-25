#pragma once

#include "boink/debugger/controller.h"
#include <piksel/window.hh>

namespace boink
{
  class CameraController : public Controller
  {
  public:
    void updateMouse(piksel::Window& wnd);
    void update(piksel::Window& wnd, piksel::Camera& camera,float dt) override;
  private:
    piksel::Window::MousePos prev_mouse_pos_;
  };
}
