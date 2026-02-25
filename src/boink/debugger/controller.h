#pragma once

#include <piksel/camera.hh>
#include <piksel/color.hh>
#include <piksel/window.hh>

namespace boink
{
  class Controller
  {
  public:
    virtual void update(piksel::Window& wnd, piksel::Camera& camera, float dt)=0;
  protected:
    Controller()=default;
  };
}
