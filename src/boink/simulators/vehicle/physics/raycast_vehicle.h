/*
 * Copyright (c) 2005 Erwin Coumans https://bulletphysics.org
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 */

// Modifications:
// 2026 - v4m3rr 
// - Refactor btRaycastVehicle
#pragma once

#include <BulletDynamics/Dynamics/btActionInterface.h>

#include "boink/simulators/vehicle/physics/vehicle_raycaster.h"
#include "boink/simulators/vehicle/physics/wheel_info.h"

namespace boink
{
  class RaycastVehicle : public btActionInterface
  {
  public:
    struct VehicleTuning
    {
    public:
      VehicleTuning()
        : m_suspensionStiffness(btScalar(5.88)),
          m_suspensionCompression(btScalar(0.83)),
          m_suspensionDamping(btScalar(0.88)),
          m_maxSuspensionTravelCm(btScalar(500.)),
          m_frictionSlip(btScalar(10.5)),
          m_maxSuspensionForce(btScalar(6000.))
      {
      }
      btScalar m_suspensionStiffness;
      btScalar m_suspensionCompression;
      btScalar m_suspensionDamping;
      btScalar m_maxSuspensionTravelCm;
      btScalar m_frictionSlip;
      btScalar m_maxSuspensionForce;
    };

  public:
    //constructor to create a car from an existing rigidbody
    RaycastVehicle(
        btRigidBody* chassis, 
        VehicleRaycaster* raycaster);
    virtual ~RaycastVehicle()=default;

    virtual void updateAction(
        btCollisionWorld* collisionWorld, btScalar step);

    void debugDraw(btIDebugDraw* debugDrawer);
    void enableDraw(bool enable=true){m_drawEnable=enable;}

    const btTransform& getChassisWorldTransform() const;
    const btTransform& getWheelTransformWS(int wheelIndex) const;
    void updateWheelTransform(
        int wheelIndex, bool interpolatedTransform = true);
    void updateWheelTransformsWS(
        WheelInfo& wheel, bool interpolatedTransform = true);

    WheelInfo& addWheel(
        const btVector3& connectionPointCS0, 
        const btVector3& wheelDirectionCS0, 
        const btVector3& wheelAxleCS, 
        btScalar suspensionRestLength, 
        btScalar wheelRadius, 
        const VehicleTuning& tuning, 
        bool isFrontWheel);

    inline int getNumWheels() const{return int(m_wheelInfo.size());}
    const WheelInfo& getWheelInfo(int index) const;
    WheelInfo& getWheelInfo(int index);

    btScalar getSteeringValue(int wheel) const;
    void setSteeringValue(btScalar steering, int wheel);
    void applyEngineForce(btScalar force, int wheel);
    void setBrake(btScalar brake, int wheelIndex);

    btScalar rayCast(WheelInfo& wheel);

    void resetSuspension();
    virtual void updateSuspension(btScalar deltaTime);
    virtual void updateFriction(btScalar timeStep);

    inline btRigidBody* getRigidBody(){return m_chassisBody;}
    const btRigidBody* getRigidBody() const{return m_chassisBody;}

    inline int getRightAxis() const {return m_indexRightAxis;}
    inline int getUpAxis() const {return m_indexUpAxis;}
    inline int getForwardAxis() const {return m_indexForwardAxis;}

    btVector3 getForwardVector() const;

    btScalar getCurrentSpeedKmHour() const{return m_currentVehicleSpeedKmHour;}

    void setPitchControl(btScalar pitch) {m_pitchControl = pitch;}

    virtual void setCoordinateSystem(
        int rightIndex, int upIndex, int forwardIndex);

    int getUserConstraintType() const {return m_userConstraintType;}
    void setUserConstraintType(int userConstraintType)
    {m_userConstraintType = userConstraintType;}

    void setUserConstraintId(int uid){m_userConstraintId = uid;}
    int getUserConstraintId() const {return m_userConstraintId;}
  private:
    void applyAerodynamics(btScalar step);
    void updateFrictionBasedOnSurface(btScalar step);
  private:
    btAlignedObjectArray<btVector3> m_forwardWS;
    btAlignedObjectArray<btVector3> m_axle;
    btAlignedObjectArray<btScalar> m_forwardImpulse;
    btAlignedObjectArray<btScalar> m_sideImpulse;
    int m_userConstraintType;
    int m_userConstraintId;

    VehicleRaycaster* m_vehicleRaycaster;
    btScalar m_pitchControl;
    btScalar m_steeringValue;
    btScalar m_currentVehicleSpeedKmHour;

    btRigidBody* m_chassisBody;

    int m_indexRightAxis;
    int m_indexUpAxis;
    int m_indexForwardAxis;

    btAlignedObjectArray<WheelInfo> m_wheelInfo;

    bool m_drawEnable=true;
  };
}
