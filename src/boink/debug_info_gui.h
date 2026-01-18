#pragma once

#include "piksel/gui_object.hh"
#include "boink/simulation/simulation_info.h"

namespace boink
{
  class DebugInfoGui : public piksel::GuiObject
  {
  public:
    ~DebugInfoGui() noexcept override=default;

    void draw() override;

    std::string_view getTitle() const override
    {
      return "Debug Info";
    }

    bool isCameraEnable() const
    {
      return enable_camera_;
    }

    float getCameraSpeed() const
    {
      return camera_speed_;
    }

    float getMouseSpeed() const
    {
      return mouse_speed_;
    }

    bool isSimulationFreeze() const
    {
      return freeze_sim_;
    }
  public:
    SimulationInfo simulation_info;
  private:
    bool freeze_sim_=true;
    bool enable_camera_=true;
    float mouse_speed_=0.3f;
    float camera_speed_=5.f;
  };
}
