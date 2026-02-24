#pragma once

#include <BulletDynamics/Vehicle/btRaycastVehicle.h>
#include <BulletDynamics/Vehicle/btWheelInfo.h>
#include <LinearMath/btIDebugDraw.h>

#include "boink/simulators/vehicle/wheel_position.h"

#include <unordered_map>

namespace boink
{
  class CustomRaycastVehicle : public btRaycastVehicle
  {
  public:
    CustomRaycastVehicle(
        const btVehicleTuning& tuning,
        btRigidBody* chassis, 
        btVehicleRaycaster* raycaster );
    virtual ~CustomRaycastVehicle() noexcept=default;

    virtual void updateAction(
        btCollisionWorld* collison_world,
        btScalar step) override;
    virtual void updateVehicle(btScalar step) override;
    virtual void updateFriction(btScalar time_step) override;
    virtual void debugDraw(btIDebugDraw* dbg) override;

    // TODO hide from btRaycast maybe do like private inheritance?
    btScalar getWheelAngularSpeed(WheelPosition pos) const
    {
      return wheel_speeds_.at(pos);
    }
  private:
    void updateWheels(btScalar step);
    void applyAerodynamics();
    void* getGroundObject(
        btWheelInfo& wheel,
        btVehicleRaycaster::btVehicleRaycasterResult& out_result);
    void updateWheelsFrictions();
  private:
    btVehicleRaycaster* p_raycaster_;
    // We need to rewrite it
    std::unordered_map<WheelPosition,btScalar> wheel_speeds_={
      {WheelPosition::FrontLeft,0.f},
      {WheelPosition::FrontRight,0.f},
      {WheelPosition::RearLeft,0.f},
      {WheelPosition::RearRight,0.f},
    };
  };
}
