/*
 * Copyright (c) 2005 Erwin Coumans http://bulletphysics.org
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/
//
// Modifications:
// 2026 - v4m3rr
// - Cosmetic changes
#pragma once

#include <LinearMath/btVector3.h>
#include <LinearMath/btTransform.h>

class btRigidBody;
namespace boink
{
  struct WheelInfoConstructionInfo
  {
    btVector3 m_chassisConnectionCS;
    btVector3 m_wheelDirectionCS;
    btVector3 m_wheelAxleCS;
    btScalar m_suspensionRestLength;
    btScalar m_maxSuspensionTravelCm;
    btScalar m_wheelRadius;

    btScalar m_suspensionStiffness;
    btScalar m_wheelsDampingCompression;
    btScalar m_wheelsDampingRelaxation;
    btScalar m_frictionSlip;
    btScalar m_maxSuspensionForce;
    btScalar m_rollInfluence=0.1f;
    bool m_bIsFrontWheel;
  };

  struct WheelInfo
  {
    struct RaycastInfo
    {
      //set by raycaster
      btVector3 m_contactNormalWS;  //contactnormal
      btVector3 m_contactPointWS;   //raycast hitpoint
      btScalar m_suspensionLength;
      btVector3 m_hardPointWS;       //raycast starting point
      btVector3 m_wheelDirectionWS;  //direction in worldspace
      btVector3 m_wheelAxleWS;       // axle in worldspace
      bool m_isInContact;
      btRigidBody* m_groundObject;
    };
    RaycastInfo m_raycastInfo;

    struct SuspensionInfo
    {
      btVector3 m_chassisConnectionPointCS;  //const
      btScalar m_restLength;      //const
      btScalar m_maxTravelCm;
      btScalar m_stiffness;       //const
      btScalar m_wheelsDampingCompression;  //const
      btScalar m_wheelsDampingRelaxation;   //const
      btScalar m_maxForce;
    };
    SuspensionInfo m_suspensionInfo;
    //why?
    btScalar getSuspensionRestLength() const
    { return m_suspensionInfo.m_restLength;}

    btVector3 m_wheelDirectionCS;          //const
    btVector3 m_wheelAxleCS;               // const or modified by steering
    btTransform m_worldTransform;
                                           
    btScalar m_wheelsRadius;              //const
    btScalar m_rotation;
    btScalar m_deltaRotation;

    btScalar m_frictionSlip;
    btScalar m_rollInfluence;

    btScalar m_engineForce;
    btScalar m_steering;
    btScalar m_brake;
 
    bool m_bIsFrontWheel;

    void* m_clientInfo;  //can be used to store pointer to sync transforms...

    btScalar m_clippedInvContactDotSuspension;
    btScalar m_suspensionRelativeVelocity;

    //calculated by suspension
    btScalar m_wheelsSuspensionForce;
    btScalar m_skidInfo;
  public:
    WheelInfo()=default;
    WheelInfo(WheelInfoConstructionInfo& ci);

    void updateWheel(const btRigidBody& chassis, RaycastInfo& raycastInfo);
  };
}
