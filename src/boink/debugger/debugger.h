#pragma once

#include <LinearMath/btVector3.h>

#include <piksel/gui_object.hh>
#include <piksel/window.hh>
#include <piksel/camera.hh>
#include <piksel/gui_manager.hh>

#include "boink/debugger/renderer.h"
#include "boink/simulators/simulator.h"
#include "boink/debugger/camera_controller.h"

#include <string_view>

namespace boink
{
  class DebuggerGui;
  class Debugger
  {
  public:
    friend class DebuggerGui;
  public:
    Debugger(
        std::string_view title,
        const btVector3& camera_position,
        const btVector3& camera_target);

    void update();
    btScalar getTime() const;
    Renderer* getRendererPtr() {return &renderer_;}
    float getFramerate() const {return fps_;}

    void addGui(std::shared_ptr<piksel::GuiObject> gui_object);
    void setControllers(
        std::vector<
          std::pair<Simulator::ID,std::shared_ptr<Controller>>> controllers);

    bool shouldClose() const;
  private:
    void handleWindowClose();
    void updateController(float dt);
    inline void calculateFramerate(float dt) {fps_=1.f/dt;}
  private:
    piksel::Window wnd_;
    piksel::Camera cam_;
    Renderer renderer_;
    piksel::GuiManager gui_manager_;

    std::shared_ptr<CameraController> default_controller_;

    std::vector<
      std::pair<Simulator::ID,std::shared_ptr<Controller>>> controllers_;

    float fps_;
    int selected_controller_=-1;
    float cam_speed_;
    float mouse_speed_;

    std::shared_ptr<DebuggerGui> gui_;
  };
}
