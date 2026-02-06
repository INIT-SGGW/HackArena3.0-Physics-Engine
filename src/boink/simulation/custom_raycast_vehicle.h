#pragma once

#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

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

  private:
    void applyAerodynamics();
  };
}
