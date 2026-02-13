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

    int getSelectedVehicleId() const
    {
      return selected_vehicle_id_;
    }

    bool isSimulationFreeze() const
    {
      return freeze_sim_;
    }

    void setFramerate(float fps) { fps_=fps;}

    btScalar getThrottleApplied() const
    {
      return throttle_applied_;
    }

    btScalar getBrakeApplied() const
    {
      return brake_applied_;
    }

    btScalar getSteeringApplied() const
    {
      return steer_applied_;
    }

    void setThrottleApplied(btScalar throttle_applied)
    {
      throttle_applied_=throttle_applied;
    }

    void setBrakeApplied(btScalar brake_applied)
    {
      brake_applied_=brake_applied;
    }

    void setSteeringApplied(btScalar sterring_applied)
    {
      steer_applied_=sterring_applied;
    }
  private:
    void selectCar();
  public:
    SimulationInfo simulation_info;
  private:
    float fps_=0.f;
    bool freeze_sim_=true;
    bool enable_camera_=true;
    float mouse_speed_=0.3f;
    float camera_speed_=5.f;

    btScalar throttle_applied_;
    btScalar brake_applied_;
    btScalar steer_applied_;

    int selected_vehicle_id_=-1;
  };
}
