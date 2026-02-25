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
#include "boink/simulators/vehicle/physics/wheel_info.h"

#include <BulletDynamics/Dynamics/btRigidBody.h>

namespace boink
{
  WheelInfo::WheelInfo(WheelInfoConstructionInfo& ci)
  {
    m_suspensionInfo.m_restLength = ci.m_suspensionRestLength;
    m_suspensionInfo.m_maxTravelCm = ci.m_maxSuspensionTravelCm;
    m_suspensionInfo.m_stiffness = ci.m_suspensionStiffness;
    m_suspensionInfo.m_wheelsDampingCompression = ci.m_wheelsDampingCompression;
    m_suspensionInfo.m_wheelsDampingRelaxation = ci.m_wheelsDampingRelaxation;
    m_suspensionInfo.m_chassisConnectionPointCS = ci.m_chassisConnectionCS;
    m_suspensionInfo.m_maxForce = ci.m_maxSuspensionForce;

    m_wheelDirectionCS = ci.m_wheelDirectionCS;
    m_wheelAxleCS = ci.m_wheelAxleCS;

    m_frictionSlip = ci.m_frictionSlip;
    m_rollInfluence = btScalar(0.1);

    m_engineForce = btScalar(0.);
    m_steering = btScalar(0.);
    m_brake = btScalar(0.);

    m_wheelsRadius = ci.m_wheelRadius;
    m_rotation = btScalar(0.);
    m_deltaRotation = btScalar(0.);

    m_bIsFrontWheel = ci.m_bIsFrontWheel;
  }

  void WheelInfo::updateWheel(
      const btRigidBody& chassis, RaycastInfo& raycastInfo)
  {
    (void)raycastInfo;

    if (m_raycastInfo.m_isInContact)
    {
      btScalar project = 
        m_raycastInfo.m_contactNormalWS.dot(
          m_raycastInfo.m_wheelDirectionWS);
      btVector3 chassis_velocity_at_contactPoint;
      btVector3 relpos = 
        m_raycastInfo.m_contactPointWS - 
        chassis.getCenterOfMassPosition();
      chassis_velocity_at_contactPoint = 
        chassis.getVelocityInLocalPoint(relpos);
      btScalar projVel = 
        m_raycastInfo.m_contactNormalWS.dot(chassis_velocity_at_contactPoint);

      if (project >= btScalar(-0.1))
      {
        m_suspensionRelativeVelocity = btScalar(0.0);
        m_clippedInvContactDotSuspension = btScalar(1.0) / btScalar(0.1);
      }
      else
      {
        btScalar inv = btScalar(-1.) / project;
        m_suspensionRelativeVelocity = projVel * inv;
        m_clippedInvContactDotSuspension = inv;
      }
    }
    else  // Not in contact : position wheel in a nice (rest length) position
    {
      m_raycastInfo.m_suspensionLength = m_suspensionInfo.m_restLength;
      m_suspensionRelativeVelocity = btScalar(0.0);
      m_raycastInfo.m_contactNormalWS = -m_raycastInfo.m_wheelDirectionWS;
      m_clippedInvContactDotSuspension = btScalar(1.0);
    }
  }
}
