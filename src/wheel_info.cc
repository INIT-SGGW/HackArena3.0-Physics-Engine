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

#include <array>

namespace boink
{
// TODO
btScalar WheelInfo::TyreInfo::heatingConst = btScalar(0.00005f);
btScalar WheelInfo::TyreInfo::coolingConst = btScalar(0.001f);
btScalar WheelInfo::TyreInfo::wearRate = btScalar(0.0000001f);

btScalar WheelInfo::TyreInfo::s_softWearRatePerMin = btScalar(0.05);
btScalar WheelInfo::TyreInfo::s_hardWearRatePerMin = s_softWearRatePerMin;
btScalar WheelInfo::TyreInfo::s_wetWearRatePerMin = s_softWearRatePerMin;

btScalar WheelInfo::TyreInfo::s_slipRatioTempConstant = btScalar(0.5);
btScalar WheelInfo::TyreInfo::s_angularSpeedTempConstant = btScalar(0.1);

btScalar WheelInfo::TyreInfo::s_angularSpeedTempCoolingConst = btScalar(0.1);

// btScalar baseDryGrip;
// btScalar baseWetGrip;
// btScalar wearRate;
// btScalar optimalTemp;
// btScalar heatingFactor;
//
// Curve tempToGripCoeff;
// Curve tempToStiffCoeff;
const std::array<TyreTypeProperties, 3> WheelInfo::kTyresTypesInfo = {
    {{1.00f, 0.2f, 3.f, 100.f, 1.2f,
      Curve({0.80, 0.82, 0.85, 0.88, 0.92, 0.96, 0.98, 0.99, 1.00, 1.00, 0.98, 0.94, 0.88, 0.82, 0.78, 0.75, 0.75},
            10.f, 0.f),
      Curve({1.50, 1.40, 1.30, 1.20, 1.15, 1.10, 1.05, 1.02, 1.00, 1.00, 0.95, 0.85, 0.75, 0.65, 0.60, 0.55, 0.50},
            10.f, 0.f)},  // soft
     {0.96f, 0.2f, 1.f, 100.f, 0.8f,
      Curve({0.75, 0.77, 0.80, 0.83, 0.86, 0.89, 0.92, 0.95, 0.97, 0.99, 1.00, 1.00, 0.99, 0.97, 0.95, 0.92, 0.88},
            10.f, 0.f),
      Curve({1.60, 1.50, 1.40, 1.30, 1.25, 1.20, 1.15, 1.10, 1.05, 1.02, 1.00, 1.00, 0.98, 0.95, 0.90, 0.85, 0.80},
            10.f, 0.f)},  // hard
     {0.85f, 0.8f, 1.5f, 75.f, 2.5f,
      Curve({0.85, 0.88, 0.92, 0.95, 0.97, 0.99, 1.00, 1.00, 0.95, 0.85, 0.75, 0.65, 0.55, 0.50, 0.50, 0.50, 0.50},
            10.f, 0.f),
      Curve({1.30, 1.20, 1.15, 1.10, 1.05, 1.02, 1.00, 1.00, 0.90, 0.75, 0.60, 0.50, 0.45, 0.40, 0.40, 0.40, 0.40},
            10.f, 0.f)}}};  // wet

WheelInfo::WheelInfo(WheelInfoConstructionInfo& ci)
{
  m_suspensionInfo.m_restLength = ci.m_suspensionRestLength;
  m_suspensionInfo.m_maxTravelCm = ci.m_maxSuspensionTravelCm;
  m_suspensionInfo.m_stiffness = ci.m_suspensionStiffness;
  m_suspensionInfo.m_wheelsDampingCompression = ci.m_wheelsDampingCompression;
  m_suspensionInfo.m_wheelsDampingRelaxation = ci.m_wheelsDampingRelaxation;
  m_suspensionInfo.m_chassisConnectionPointCS = ci.m_chassisConnectionCS;
  m_suspensionInfo.m_maxForce = ci.m_maxSuspensionForce;

  m_tyreInfo.m_health = btScalar(1.0);
  m_tyreInfo.m_type = ci.m_tyreType;

  m_wheelDirectionCS = ci.m_wheelDirectionCS;
  m_wheelAxleCS = ci.m_wheelAxleCS;

  m_frictionSlip = ci.m_frictionSlip;
  m_rollInfluence = btScalar(0.1);
  m_slipRatio = btScalar(0.0);

  m_engineForce = btScalar(0.);
  m_steering = btScalar(0.);
  m_brake = btScalar(0.);

  m_wheelsRadius = ci.m_wheelRadius;
  m_wheelSimRadius = btScalar(0.33);
  // m_rotation = btScalar(0.);
  // m_deltaRotation = btScalar(0.);

  m_bIsFrontWheel = ci.m_bIsFrontWheel;
}

void WheelInfo::updateWheel(const btRigidBody& chassis, RaycastInfo& raycastInfo)
{
  (void)raycastInfo;

  if (m_raycastInfo.m_isInContact)
  {
    btScalar project = m_raycastInfo.m_contactNormalWS.dot(m_raycastInfo.m_wheelDirectionWS);
    btVector3 chassis_velocity_at_contactPoint;
    btVector3 relpos = m_raycastInfo.m_contactPointWS - chassis.getCenterOfMassPosition();
    chassis_velocity_at_contactPoint = chassis.getVelocityInLocalPoint(relpos);
    btScalar projVel = m_raycastInfo.m_contactNormalWS.dot(chassis_velocity_at_contactPoint);

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
}  // namespace boink
