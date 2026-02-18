#include "boink/simulators/vehicle/custom_raycast_vehicle.h"

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

#include <BulletDynamics/Vehicle/btWheelInfo.h>

#include "boink/simulators/track/ground.h"
#include <cassert>

namespace boink
{
  CustomRaycastVehicle::CustomRaycastVehicle(
      const btVehicleTuning& tuning,
      btRigidBody* chassis, 
      btVehicleRaycaster* raycaster )
    :btRaycastVehicle(tuning,chassis,raycaster),
    p_raycaster_(raycaster)
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
    this->updateWheelsFrictions();
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

    constexpr btScalar kAirDensity=1.225f;
    constexpr btScalar kAirDragCoef= 1.f;
    constexpr btScalar kFrontalArea= 1.4f;

    btVector3 air_drag_force= 
      -0.5f*kAirDragCoef*kFrontalArea*kAirDensity*
      speed*speed*vel_dir;

    constexpr btScalar kAirLiftCoef=kAirDragCoef*2.5f;

    btVector3 down_dir=-rigidbody->getWorldTransform().getBasis().getColumn(1);

    assert(down_dir.length()<1.01&&down_dir.length()>0.99);

    btVector3 air_down_force=
      0.5f*kAirLiftCoef*kFrontalArea*kAirDensity*
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

  void* CustomRaycastVehicle::getGroundObject(btWheelInfo& wheel)
  {
      btScalar raylen = wheel.getSuspensionRestLength() + wheel.m_wheelsRadius;

      btVector3 rayvector = wheel.m_raycastInfo.m_wheelDirectionWS * (raylen);
      const btVector3& source = wheel.m_raycastInfo.m_hardPointWS;
      btVector3 target = source+rayvector;

      btVehicleRaycaster::btVehicleRaycasterResult rayResults;

      btAssert(p_raycaster_);

      void* object = p_raycaster_->castRay(source, target, rayResults);
      return object;
  }

  void CustomRaycastVehicle::updateWheelsFrictions()
  {
    for(int i=0;i<this->getNumWheels();i++)
    {
      btWheelInfo& wheel=this->getWheelInfo(i);
      void* p_ground=this->getGroundObject(wheel);

      if(!p_ground)
        return;

      btRigidBody* ground_rb=(btRigidBody*)p_ground;
      Ground::SurfaceInfo& surface_info=
        *(Ground::SurfaceInfo*)(ground_rb->getUserPointer());

      wheel.m_frictionSlip=surface_info.friction;
      wheel.m_rollInfluence=surface_info.rolling_resistance;
    }
  }
}
