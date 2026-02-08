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

  void CustomRaycastVehicle::debugDraw(btIDebugDraw* dbg)
  {
    //btRaycastVehicle::debugDraw(dbg);
    for (int v = 0; v < this->getNumWheels(); v++)
    {
      btVector3 wheelColor;
      if (getWheelInfo(v).m_raycastInfo.m_isInContact)
        wheelColor.setValue(0, 0, 1);
      else
        wheelColor.setValue(1, 0, 1);

      btVector3 wheelPosWS = getWheelInfo(v).m_worldTransform.getOrigin();

      btVector3 axle = btVector3(
              getWheelInfo(v).m_worldTransform.getBasis()[0][getRightAxis()],
              getWheelInfo(v).m_worldTransform.getBasis()[1][getRightAxis()],
              getWheelInfo(v).m_worldTransform.getBasis()[2][getRightAxis()]);

      dbg->drawLine(wheelPosWS, wheelPosWS + axle, wheelColor);
      dbg->drawLine(
          wheelPosWS, 
          getWheelInfo(v).m_raycastInfo.m_contactPointWS, 
          wheelColor);

      // Draw suspension
      dbg->drawLine(
          wheelPosWS,
          getWheelInfo(v).m_raycastInfo.m_hardPointWS,
          {1,0,0});
    }      
  }
}
