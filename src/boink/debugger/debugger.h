#pragma once

#include <LinearMath/btVector3.h>

#include <piksel/window.hh>
#include <piksel/camera.hh>

#include "boink/debugger/renderer.h"

#include <string_view>

namespace boink
{
  class Debugger
  {
  public:
    Debugger(
        std::string_view title,
        const btVector3& camera_position,
        const btVector3& camera_target);

    void update();
    btScalar getTime() const;
    Renderer* getRendererPtr() {return &renderer_;}
    float getFramerate() const {return fps_;}

    bool shouldClose() const;
  private:
    void handleWindowClose();
    void handleCameraMovement(float dt);
    inline void calculateFramerate(float dt) {fps_=1.f/dt;}
  private:
    piksel::Window wnd_;
    piksel::Camera cam_;
    Renderer renderer_;

    float fps_;
  };
}
