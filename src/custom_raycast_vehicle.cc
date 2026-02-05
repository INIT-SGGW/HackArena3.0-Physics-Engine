#include "boink/simulation/custom_raycast_vehicle.h"
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

namespace boink
{
  CustomRaycastVehicle::CustomRaycastVehicle(
      const btVehicleTuning& tuning,
      btRigidBody* chassis, 
      btVehicleRaycaster* raycaster )
    :btRaycastVehicle(tuning,chassis,raycaster)
  {
  }

  void CustomRaycastVehicle::updateAction(
      btCollisionWorld* collision_world,
      btScalar step)
  {
    (void)collision_world;

    this->updateVehicle(step);
  }

  void CustomRaycastVehicle::updateVehicle(btScalar step)
  {
    btRaycastVehicle::updateVehicle(step);

    this->applyAerodynamics();
  }

  void CustomRaycastVehicle::updateFriction(btScalar time_step)
  {
    btRaycastVehicle::updateFriction(time_step);
    (void)time_step;
  }

  void CustomRaycastVehicle::applyAerodynamics()
  {
    auto rigidbody=this->getRigidBody();
    const btVector3& velocity=rigidbody->getLinearVelocity();
    const btScalar speed=velocity.length();

    if(speed < 0.1)
      return;

    btVector3 vel_dir=velocity/speed;

    constexpr btScalar kAirDensity=1.225;
    constexpr btScalar kAirDragCoef=1.; 
    constexpr btScalar kFrontalArea=1.4; 

    btVector3 air_drag_force= 
      -0.5*kAirDragCoef*kFrontalArea*kAirDensity*
      speed*speed*vel_dir;

    constexpr btScalar kAirLiftCoef=kAirDragCoef*2.5;

    btVector3 down_dir=-rigidbody->getWorldTransform().getBasis().getColumn(1);

    assert(down_dir.length()<1.01&&down_dir.length()>0.99);

    btVector3 air_down_force=
      0.5*kAirLiftCoef*kFrontalArea*kAirDensity*
      speed*speed*down_dir;

    rigidbody->applyCentralForce(air_drag_force+air_down_force);
  }
}
