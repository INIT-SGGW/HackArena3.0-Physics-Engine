#pragma once

#include <LinearMath/btIDebugDraw.h>

#include "boink/simulators/vehicle/wheel_position.h"
#include "boink/simulators/vehicle/physics/raycast_vehicle.h"

#include <unordered_map>

namespace boink
{
  class CustomRaycastVehicle : public RaycastVehicle
  {
  public:
    CustomRaycastVehicle(
        btRigidBody* chassis, 
        VehicleRaycaster* raycaster );
    virtual ~CustomRaycastVehicle() noexcept=default;

    virtual void updateAction(
        btCollisionWorld* collison_world,
        btScalar step) override;
    virtual void updateFriction(btScalar time_step) override;
    virtual void debugDraw(btIDebugDraw* dbg) override;

    // TODO hide from btRaycast maybe do like private inheritance?
    btScalar getWheelAngularSpeed(WheelPosition pos) const
    {
      return wheel_speeds_.at(pos);
    }

    void enableDraw(bool enable=true)
    {
      draw_enable=enable;
    }
  private:
    void updateWheels(btScalar step);
    void applyAerodynamics();
    void* getGroundObject(
        WheelInfo& wheel,
        VehicleRaycaster::VehicleRaycasterResult& out_result);
    void updateWheelsFrictions();
  private:
    VehicleRaycaster* p_raycaster_;
    // We need to rewrite it
    std::unordered_map<WheelPosition,btScalar> wheel_speeds_={
      {WheelPosition::FrontLeft,0.f},
      {WheelPosition::FrontRight,0.f},
      {WheelPosition::RearLeft,0.f},
      {WheelPosition::RearRight,0.f},
    };

    bool draw_enable=true;
  };
}
