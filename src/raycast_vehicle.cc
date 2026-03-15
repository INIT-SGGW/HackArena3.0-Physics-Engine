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
#include "boink/simulators/vehicle/physics/raycast_vehicle.h"

#include <BulletDynamics/ConstraintSolver/btContactConstraint.h>
#include <BulletDynamics/ConstraintSolver/btJacobianEntry.h>
#include <BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <LinearMath/btIDebugDraw.h>
#include <LinearMath/btMinMax.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>

// #include <iostream>

#include "boink/bullet_user_data.h"
#include "boink/simulators/track/ground.h"
#include "boink/simulators/vehicle/physics/vehicle_raycaster.h"
#include "boink/simulators/vehicle/physics/wheel_info.h"
#include "boink/simulators/vehicle/wheel_position.h"

#define ROLLING_INFLUENCE_FIX

btRigidBody& btActionInterface::getFixedBody()
{
  static btRigidBody s_fixed(0, 0, 0);
  s_fixed.setMassProps(btScalar(0.), btVector3(btScalar(0.), btScalar(0.), btScalar(0.)));
  return s_fixed;
}

namespace boink
{
RaycastVehicle::RaycastVehicle(btRigidBody* chassis, VehicleRaycaster* raycaster)
    : m_vehicleRaycaster(raycaster), m_pitchControl(btScalar(0.))
{
  m_chassisBody = chassis;
  m_indexRightAxis = 0;
  m_indexUpAxis = 2;
  m_indexForwardAxis = 1;

  m_currentVehicleSpeedKmHour = btScalar(0.);
  m_steeringValue = btScalar(0.);
}

void RaycastVehicle::updateAction(btCollisionWorld* collisionWorld, btScalar step)
{
  (void)collisionWorld;

  for (int i = 0; i < getNumWheels(); i++)
  {
    updateWheelTransform(i, false);
  }

  m_currentVehicleSpeedKmHour = btScalar(3.6) * getRigidBody()->getLinearVelocity().length();

  const btTransform& chassisTrans = getChassisWorldTransform();

  btVector3 forwardW(chassisTrans.getBasis()[0][m_indexForwardAxis], chassisTrans.getBasis()[1][m_indexForwardAxis],
                     chassisTrans.getBasis()[2][m_indexForwardAxis]);

  if (forwardW.dot(getRigidBody()->getLinearVelocity()) < btScalar(0.)) m_currentVehicleSpeedKmHour *= btScalar(-1.);

  applyAerodynamics(step);

  //
  // simulate suspension
  //
  for (int i = 0; i < m_wheelsInfo.size(); i++) rayCast(m_wheelsInfo[i]);

  updateSuspension(step);

  for (int i = 0; i < m_wheelsInfo.size(); i++)
  {
    // apply suspension force
    WheelInfo& wheel = m_wheelsInfo[i];

    btScalar suspensionForce = wheel.m_wheelsSuspensionForce;
    if (suspensionForce > wheel.m_suspensionInfo.m_maxForce)
      wheel.m_wheelsSuspensionForce = wheel.m_suspensionInfo.m_maxForce;

    btVector3 impulse = wheel.m_raycastInfo.m_contactNormalWS * suspensionForce * step;
    btVector3 relpos = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();

    getRigidBody()->applyImpulse(impulse, relpos);
  }

  updateFriction(step);
}

const btTransform& RaycastVehicle::getChassisWorldTransform() const
{
  // if (getRigidBody()->getMotionState())
  //{
  //   btTransform chassisWorldTrans;
  //   getRigidBody()->getMotionState()->getWorldTransform(chassisWorldTrans);
  //   return chassisWorldTrans;
  // }

  return getRigidBody()->getCenterOfMassTransform();
}

const btTransform& RaycastVehicle::getWheelTransformWS(int wheelIndex) const
{
  btAssert(wheelIndex < getNumWheels());

  const WheelInfo& wheel = m_wheelsInfo[wheelIndex];
  return wheel.m_worldTransform;
}

void RaycastVehicle::updateWheelTransform(int wheelIndex, bool interpolatedTransform)
{
  WheelInfo& wheel = m_wheelsInfo[wheelIndex];
  updateWheelTransformsWS(wheel, interpolatedTransform);
  btVector3 up = -wheel.m_raycastInfo.m_wheelDirectionWS;
  const btVector3& right = wheel.m_raycastInfo.m_wheelAxleWS;
  btVector3 fwd = up.cross(right);
  fwd = fwd.normalize();
  //      up = right.cross(fwd);
  //      up.normalize();

  // rotate around steering over de wheelAxleWS
  btScalar steering = wheel.m_steering;

  btQuaternion steeringOrn(up, steering);  // wheel.m_steering);
  btMatrix3x3 steeringMat(steeringOrn);

  btQuaternion rotatingOrn(right, -wheel.m_rotation);
  btMatrix3x3 rotatingMat(rotatingOrn);

  btMatrix3x3 basis2;
  basis2[0][m_indexRightAxis] = -right[0];
  basis2[1][m_indexRightAxis] = -right[1];
  basis2[2][m_indexRightAxis] = -right[2];

  basis2[0][m_indexUpAxis] = up[0];
  basis2[1][m_indexUpAxis] = up[1];
  basis2[2][m_indexUpAxis] = up[2];

  basis2[0][m_indexForwardAxis] = fwd[0];
  basis2[1][m_indexForwardAxis] = fwd[1];
  basis2[2][m_indexForwardAxis] = fwd[2];

  wheel.m_worldTransform.setBasis(steeringMat * rotatingMat * basis2);
  wheel.m_worldTransform.setOrigin(wheel.m_raycastInfo.m_hardPointWS +
                                   wheel.m_raycastInfo.m_wheelDirectionWS * wheel.m_raycastInfo.m_suspensionLength);
}

void RaycastVehicle::updateWheelTransformsWS(WheelInfo& wheel, bool interpolatedTransform)
{
  wheel.m_raycastInfo.m_isInContact = false;

  btTransform chassisTrans = getChassisWorldTransform();
  if (interpolatedTransform && (getRigidBody()->getMotionState()))
    getRigidBody()->getMotionState()->getWorldTransform(chassisTrans);

  wheel.m_raycastInfo.m_hardPointWS = chassisTrans(wheel.m_suspensionInfo.m_chassisConnectionPointCS);
  wheel.m_raycastInfo.m_wheelDirectionWS = chassisTrans.getBasis() * wheel.m_wheelDirectionCS;
  wheel.m_raycastInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel.m_wheelAxleCS;
}

WheelInfo& RaycastVehicle::addWheel(const btVector3& connectionPointCS, const btVector3& wheelDirectionCS0,
                                    const btVector3& wheelAxleCS, btScalar suspensionRestLength, btScalar wheelRadius,
                                    const VehicleTuning& tuning, bool isFrontWheel, WheelInfo::TyreType tyreType)
{
  WheelInfoConstructionInfo ci;

  ci.m_chassisConnectionCS = connectionPointCS;
  ci.m_wheelDirectionCS = wheelDirectionCS0;
  ci.m_wheelAxleCS = wheelAxleCS;
  ci.m_suspensionRestLength = suspensionRestLength;
  ci.m_wheelRadius = wheelRadius;
  ci.m_suspensionStiffness = tuning.m_suspensionStiffness;
  ci.m_wheelsDampingCompression = tuning.m_suspensionCompression;
  ci.m_wheelsDampingRelaxation = tuning.m_suspensionDamping;
  ci.m_frictionSlip = tuning.m_frictionSlip;
  ci.m_bIsFrontWheel = isFrontWheel;
  ci.m_maxSuspensionTravelCm = tuning.m_maxSuspensionTravelCm;
  ci.m_maxSuspensionForce = tuning.m_maxSuspensionForce;

  ci.m_tyreType = tyreType;

  m_wheelsInfo.push_back(WheelInfo(ci));

  WheelInfo& wheel = m_wheelsInfo[getNumWheels() - 1];

  updateWheelTransformsWS(wheel, false);
  updateWheelTransform(getNumWheels() - 1, false);
  return wheel;
}

const WheelInfo& RaycastVehicle::getWheelInfo(int index) const
{
  btAssert((index >= 0) && (index < getNumWheels()));

  return m_wheelsInfo[index];
}

WheelInfo& RaycastVehicle::getWheelInfo(int index)
{
  btAssert((index >= 0) && (index < getNumWheels()));

  return m_wheelsInfo[index];
}

btScalar RaycastVehicle::getEngineRPM() const { return m_engine.rpm; }

int RaycastVehicle::getCurrentGear() const { return static_cast<int>(m_gearbox.current_gear); }

btScalar RaycastVehicle::getSteeringValue(int wheel) const { return getWheelInfo(wheel).m_steering; }

void RaycastVehicle::setSteeringValue(btScalar steering, int wheel)
{
  btAssert(wheel >= 0 && wheel < getNumWheels());

  WheelInfo& wheelInfo = getWheelInfo(wheel);
  wheelInfo.m_steering = steering;
}

void RaycastVehicle::setBrake(btScalar brake)
{
  // TODO: for now for all wheels there is always the same braking value, if it will not changed brake var in every
  // wheel is not needed
  for (int i = 0; i < m_wheelsInfo.size(); i++)
  {
    getWheelInfo(i).m_brake = brake;
  }
}

btScalar RaycastVehicle::rayCast(WheelInfo& wheel)
{
  updateWheelTransformsWS(wheel, false);

  btScalar depth = -1;

  btScalar raylen = wheel.getSuspensionRestLength() + wheel.m_wheelsRadius;

  btVector3 rayvector = wheel.m_raycastInfo.m_wheelDirectionWS * (raylen);
  const btVector3& source = wheel.m_raycastInfo.m_hardPointWS;
  wheel.m_raycastInfo.m_contactPointWS = source + rayvector;
  const btVector3& target = wheel.m_raycastInfo.m_contactPointWS;

  btScalar param = btScalar(0.);

  VehicleRaycaster::VehicleRaycasterResult rayResults;

  btAssert(m_vehicleRaycaster);

  btRigidBody* object = m_vehicleRaycaster->castRay(source, target, rayResults);

  wheel.m_raycastInfo.m_groundObject = 0;
  if (object)
  {
    param = rayResults.m_distFraction;
    depth = raylen * rayResults.m_distFraction;
    wheel.m_raycastInfo.m_contactNormalWS = rayResults.m_hitNormalInWorld;
    wheel.m_raycastInfo.m_isInContact = true;

    if (object->isStaticObject())
      wheel.m_raycastInfo.m_groundObject = object;
    else
      wheel.m_raycastInfo.m_groundObject = &getFixedBody();

    btScalar hitDistance = param * raylen;
    wheel.m_raycastInfo.m_suspensionLength = hitDistance - wheel.m_wheelsRadius;

    // clamp on max suspension travel
    btScalar minSuspensionLength =
        wheel.getSuspensionRestLength() - wheel.m_suspensionInfo.m_maxTravelCm * btScalar(0.01);
    btScalar maxSuspensionLength =
        wheel.getSuspensionRestLength() + wheel.m_suspensionInfo.m_maxTravelCm * btScalar(0.01);
    if (wheel.m_raycastInfo.m_suspensionLength < minSuspensionLength)
      wheel.m_raycastInfo.m_suspensionLength = minSuspensionLength;
    if (wheel.m_raycastInfo.m_suspensionLength > maxSuspensionLength)
      wheel.m_raycastInfo.m_suspensionLength = maxSuspensionLength;

    wheel.m_raycastInfo.m_contactPointWS = rayResults.m_hitPointInWorld;

    btScalar denominator = wheel.m_raycastInfo.m_contactNormalWS.dot(wheel.m_raycastInfo.m_wheelDirectionWS);

    btVector3 chassis_velocity_at_contactPoint;
    btVector3 relpos = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();

    chassis_velocity_at_contactPoint = getRigidBody()->getVelocityInLocalPoint(relpos);

    btScalar projVel = wheel.m_raycastInfo.m_contactNormalWS.dot(chassis_velocity_at_contactPoint);

    if (denominator >= btScalar(-0.1))
    {
      wheel.m_suspensionRelativeVelocity = btScalar(0.0);
      wheel.m_clippedInvContactDotSuspension = btScalar(1.0) / btScalar(0.1);
    }
    else
    {
      btScalar inv = btScalar(-1.) / denominator;
      wheel.m_suspensionRelativeVelocity = projVel * inv;
      wheel.m_clippedInvContactDotSuspension = inv;
    }
  }
  else
  {
    // put wheel info as in rest position
    wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
    wheel.m_suspensionRelativeVelocity = btScalar(0.0);
    wheel.m_raycastInfo.m_contactNormalWS = -wheel.m_raycastInfo.m_wheelDirectionWS;
    wheel.m_clippedInvContactDotSuspension = btScalar(1.0);
  }

  return depth;
}

void RaycastVehicle::resetSuspension()
{
  for (int i = 0; i < m_wheelsInfo.size(); i++)
  {
    WheelInfo& wheel = m_wheelsInfo[i];
    wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
    wheel.m_suspensionRelativeVelocity = btScalar(0.0);

    wheel.m_raycastInfo.m_contactNormalWS = -wheel.m_raycastInfo.m_wheelDirectionWS;
    // wheel_info.setContactFriction(btScalar(0.0));
    wheel.m_clippedInvContactDotSuspension = btScalar(1.0);
  }
}

void RaycastVehicle::updateSuspension(btScalar deltaTime)
{
  (void)deltaTime;

  btScalar chassisMass = btScalar(1.) / m_chassisBody->getInvMass();

  for (int w_it = 0; w_it < getNumWheels(); w_it++)
  {
    WheelInfo& wheel_info = m_wheelsInfo[w_it];

    if (wheel_info.m_raycastInfo.m_isInContact)
    {
      btScalar force;
      // Spring
      {
        btScalar susp_length = wheel_info.getSuspensionRestLength();
        btScalar current_length = wheel_info.m_raycastInfo.m_suspensionLength;

        btScalar length_diff = (susp_length - current_length);

        force = wheel_info.m_suspensionInfo.m_stiffness * length_diff * wheel_info.m_clippedInvContactDotSuspension;
      }

      // Damper
      {
        btScalar projected_rel_vel = wheel_info.m_suspensionRelativeVelocity;
        btScalar susp_damping;
        if (projected_rel_vel < btScalar(0.0))
          susp_damping = wheel_info.m_suspensionInfo.m_wheelsDampingCompression;
        else
          susp_damping = wheel_info.m_suspensionInfo.m_wheelsDampingRelaxation;
        force -= susp_damping * projected_rel_vel;
      }

      // RESULT
      {
        wheel_info.m_wheelsSuspensionForce = force * chassisMass;
        if (wheel_info.m_wheelsSuspensionForce < btScalar(0.)) wheel_info.m_wheelsSuspensionForce = btScalar(0.);
      }
    }
    else
    {
      wheel_info.m_wheelsSuspensionForce = btScalar(0.0);
    }
  }
}

struct WheelContactPoint
{
  btRigidBody* m_body0;
  btRigidBody* m_body1;
  btVector3 m_frictionPositionWorld;
  btVector3 m_frictionDirectionWorld;
  btScalar m_jacDiagABInv;
  btScalar m_maxImpulse;

  WheelContactPoint(btRigidBody* body0, btRigidBody* body1, const btVector3& frictionPosWorld,
                    const btVector3& frictionDirectionWorld, btScalar maxImpulse)
      : m_body0(body0),
        m_body1(body1),
        m_frictionPositionWorld(frictionPosWorld),
        m_frictionDirectionWorld(frictionDirectionWorld),
        m_maxImpulse(maxImpulse)
  {
    btScalar denom0 = body0->computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
    btScalar denom1 = body1->computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
    btScalar relaxation = 1.f;
    m_jacDiagABInv = relaxation / (denom0 + denom1);
  }
};

void RaycastVehicle::updateFriction(btScalar timeStep)
{
  /*updateFrictionBasedOnSurface(timeStep);
  updateTyres(timeStep);*/

  // std::cout << "vel_X:  " << getRigidBody()->getLinearVelocity().getX() << "\t";
  // std::cout << "vel_Y:  " << getRigidBody()->getLinearVelocity().getY() << "\t";
  // std::cout << "vel_Z:  " << getRigidBody()->getLinearVelocity().getZ() << "\t";

  int numWheel = getNumWheels();
  if (!numWheel) return;

  m_forwardWS.resize(numWheel);
  m_axle.resize(numWheel);
  m_forwardImpulse.resize(numWheel);
  m_sideImpulse.resize(numWheel);

  auto total_drive_torque = updateDriveParts(timeStep);

  // std::cout << "steer_val:  " << m_steeringValue << "\n";

  for (int wheel_idx = 0; wheel_idx < getNumWheels(); wheel_idx++)
  {
    WheelInfo& wheelInfo = m_wheelsInfo[wheel_idx];
    class btRigidBody* groundObject = (class btRigidBody*)wheelInfo.m_raycastInfo.m_groundObject;

    // @update m_axle and m_forwardWS
    // its project axle to be always parallel to the ground, so the impulses are not applied up or down
    const btTransform& wheelTrans = getWheelTransformWS(wheel_idx);

    btMatrix3x3 wheelBasis0 = wheelTrans.getBasis();
    m_axle[wheel_idx] = -btVector3(wheelBasis0[0][m_indexRightAxis], wheelBasis0[1][m_indexRightAxis],
                                   wheelBasis0[2][m_indexRightAxis]);

    const btVector3& surfNormalWS = wheelInfo.m_raycastInfo.m_contactNormalWS;
    btScalar proj = m_axle[wheel_idx].dot(surfNormalWS);
    m_axle[wheel_idx] -= surfNormalWS * proj;
    m_axle[wheel_idx] = m_axle[wheel_idx].normalize();

    m_forwardWS[wheel_idx] = surfNormalWS.cross(m_axle[wheel_idx]);
    m_forwardWS[wheel_idx].normalize();

    // @wheels forces and angular speedes
    btScalar total_torque = 0.f;
    auto traction_torque = 0.f;
    auto drive_torque = 0.f;
    auto brake_torque = (m_gearbox.current_gear != Gear::Reverse) ? -kBrakeTorque : kBrakeTorque;

    if (groundObject) traction_torque = -wheelInfo.m_traction_force * wheelInfo.m_wheelSimRadius;
    brake_torque *= wheelInfo.m_brake;

    if (!wheelInfo.m_bIsFrontWheel)
    {
      // TODO: differential here should seperate in right proportions drive torque to left and right wheel, for now is
      // always equal
      drive_torque = total_drive_torque / 2;
      total_torque = drive_torque + traction_torque + brake_torque;
    }
    else
      total_torque = traction_torque + brake_torque;

    auto wheel_inertia = wheelInfo.kWheelMass * wheelInfo.m_wheelSimRadius * wheelInfo.kWheelMassDistCoeff;
    auto engine_inertia_part = 0.f;
    if (!wheelInfo.m_bIsFrontWheel)
      engine_inertia_part =
          m_engine.inertia *
          btPow(m_gearbox.GetCurrentRatio() * m_gearbox.kDifferentialRatio * kTransmissionEfficiency, 2);

    auto wheel_angular_acceleration = total_torque / (wheel_inertia + engine_inertia_part);
    auto wheel_speed_diff = wheel_angular_acceleration * timeStep;

    if (m_gearbox.current_gear != Gear::Reverse)
    {
      if ((wheelInfo.m_angSpeed + wheel_speed_diff) < 0)
        wheelInfo.m_angSpeed = 0;
      else
        wheelInfo.m_angSpeed += wheel_speed_diff;
    }
    else
    {
      if ((wheelInfo.m_angSpeed + wheel_speed_diff) > 0)
        wheelInfo.m_angSpeed = 0;
      else
        wheelInfo.m_angSpeed += wheel_speed_diff;
    }

    auto slip_ratio = 0.0f;
    auto slip_angle = 0.f;
    m_sideImpulse[wheel_idx] = btScalar(0.);
    m_forwardImpulse[wheel_idx] = btScalar(0.);
    if (groundObject)
    {
      auto long_speed = getWheelLongSpeed(wheelInfo);
      // @slip angle
      auto speed_SA = btMax(btFabs(long_speed), btScalar(2.f));
      auto lat_speed = getWheelLatSpeed(wheelInfo);

      if (btFabs(lat_speed) < 0.00001f) lat_speed = 0.f;
      slip_angle = btAtan2(lat_speed, speed_SA);

      // std::cout << "slip_angle:  " << slip_angle << "\n";

      // @slip ratio
      auto speed_SR = long_speed;
      if (btFabs(speed_SR) < 0.00001f) speed_SR = 0.f;

      if (speed_SR == 0 && m_throttle == 0)
        wheelInfo.m_traction_force = 0.0f;
      else
      {
        if (speed_SR == 0)
        {
          if (!wheelInfo.m_bIsFrontWheel)
          {
            if (m_gearbox.current_gear == Gear::Reverse)
              slip_ratio = -0.0001f;
            else if (m_gearbox.current_gear != Gear::Neutral)
              slip_ratio = 0.0001f;
          }
        }
        else if (btFabs(speed_SR) < 2.9)
        {
          slip_ratio = (wheelInfo.m_angSpeed * wheelInfo.m_wheelSimRadius - speed_SR) / 2.9;
        }
        else
        {
          slip_ratio = (wheelInfo.m_angSpeed * wheelInfo.m_wheelSimRadius - speed_SR) / btFabs(speed_SR);
        }
      }

      // if (wheelInfo.m_bIsFrontWheel)
      //{
      // std::cout << "drive_torque:  " << drive_torque << "\t";
      // std::cout << "traction_torque: " << traction_torque << "\t";  // on old traction force
      // std::cout << "total_torque: " << total_torque << "\t";
      // std::cout << "ang_speed: " << wheelInfo.m_angSpeed << "\t";
      // std::cout << "long_speed: " << speed_SR << "\t";
      // std::cout << "lat_speed: " << lat_speed << "\t";
      // std::cout << "slip_ratio: " << slip_ratio << "\t";
      // std::cout << "slip_angle: " << slip_angle << "\t";
      // std::cout << "suspension_force: " << wheelInfo.m_wheelsSuspensionForce << "\t";
      // std::cout << "traction_force: " << wheelInfo.m_traction_force << "\n";
      //}

      // @longtudial and lateral grip distribution
      btScalar lateral_force = btScalar(0.f);
      btScalar traction_force = btScalar(0.f);

      auto slip_ang_normalized = slip_angle / kSlipAnglePeak;
      auto slip_ratio_normalized = slip_ratio / kSlipRatioPeak;

      auto slip_vec_len =
          btSqrt(slip_ang_normalized * slip_ang_normalized + slip_ratio_normalized * slip_ratio_normalized);

      if (slip_vec_len > 0.f)
      {
        auto slip_ang_scaled = slip_vec_len * kSlipAnglePeak;
        auto slip_ratio_scaled = slip_vec_len * kSlipRatioPeak;

        auto lat_grip_base = kSlipAngleToGrip.GetValue(slip_ang_scaled);
        auto long_grip_base = kSlipRatioToGrip.GetValue(slip_ratio_scaled);

        auto lat_grip = lat_grip_base * (slip_ang_normalized / slip_vec_len);
        auto long_grip = long_grip_base * (slip_ratio_normalized / slip_vec_len);

        lateral_force = -lat_grip * wheelInfo.m_wheelsSuspensionForce;
        traction_force = long_grip * wheelInfo.m_wheelsSuspensionForce;

        if (wheelInfo.m_bIsFrontWheel)
          traction_force = wheelInfo.m_traction_force +
                           kSmoothingTractionForceFactor * (traction_force - wheelInfo.m_traction_force);
        wheelInfo.m_traction_force = traction_force;

        m_sideImpulse[wheel_idx] = lateral_force * timeStep;
        m_forwardImpulse[wheel_idx] = traction_force * timeStep;
      }

      // @apply the impulses
      btVector3 rel_pos = wheelInfo.m_raycastInfo.m_contactPointWS - m_chassisBody->getCenterOfMassPosition();

      /*std::cout << "side_imp: " << m_sideImpulse[wheel_idx] << "\t";
      std::cout << "forw_imp: " << m_forwardImpulse[wheel_idx] << "\t";*/

      if (m_forwardImpulse[wheel_idx] != btScalar(0.))
        m_chassisBody->applyImpulse(m_forwardWS[wheel_idx] * (m_forwardImpulse[wheel_idx]), rel_pos);
      if (m_sideImpulse[wheel_idx] != btScalar(0.))
      {
        btVector3 rel_pos2 = wheelInfo.m_raycastInfo.m_contactPointWS - groundObject->getCenterOfMassPosition();

        btVector3 sideImp = m_axle[wheel_idx] * m_sideImpulse[wheel_idx];

        // fix. It only worked if car's up was along Y - VT.
#if defined ROLLING_INFLUENCE_FIX
        btVector3 vChassisWorldUp = getRigidBody()->getCenterOfMassTransform().getBasis().getColumn(m_indexUpAxis);
        rel_pos -= vChassisWorldUp * (vChassisWorldUp.dot(rel_pos) * (1.f - wheelInfo.m_rollInfluence));
#else
        rel_pos[m_indexUpAxis] *= wheelInfo.m_rollInfluence;
#endif
        m_chassisBody->applyImpulse(sideImp, rel_pos);

        // apply friction impulse on the ground
        groundObject->applyImpulse(-sideImp, rel_pos2);
      }
    }
    else
      wheelInfo.m_traction_force = 0.0f;
  }

  // @feedback to engine
  auto avg_ang_speed = (m_wheelsInfo[static_cast<uint8_t>(WheelPosition::RearLeft)].m_angSpeed +
                        m_wheelsInfo[static_cast<uint8_t>(WheelPosition::RearRight)].m_angSpeed) /
                       2;
  auto new_rpm =
      avg_ang_speed * m_gearbox.GetCurrentRatio() * Gearbox::kDifferentialRatio * (60.0f / (2.0f * 3.14159f));
  if (new_rpm > 15000.0f)  // rev limiter
    m_engine.is_revLimiter_active = true;
  else if (m_engine.is_revLimiter_active && new_rpm < 14600.0f)
    m_engine.is_revLimiter_active = false;
  m_engine.SetNewRPM(new_rpm);

  // std::cout << "gear:  " << m_gearbox.current_gear << "\t";
  // std::cout << "rpm:  " << m_engine.rpm << "\t";
  // std::cout << "speed: " << getRigidBody()->getLinearVelocity().length() << "\n\n";
  //   std::cout << "\n";
}

void RaycastVehicle::setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex)
{
  m_indexRightAxis = rightIndex;
  m_indexUpAxis = upIndex;
  m_indexForwardAxis = forwardIndex;
}

void RaycastVehicle::debugDraw(btIDebugDraw* debugDrawer)
{
  if (!m_drawEnable) return;

  for (int v = 0; v < this->getNumWheels(); v++)
  {
    btVector3 wheelColor(0, 1, 1);
    if (getWheelInfo(v).m_raycastInfo.m_isInContact)
      wheelColor.setValue(0, 0, 1);
    else
      wheelColor.setValue(1, 0, 1);

    btVector3 wheelPosWS = getWheelInfo(v).m_worldTransform.getOrigin();
    btVector3 axle = btVector3(getWheelInfo(v).m_worldTransform.getBasis()[0][getRightAxis()],
                               getWheelInfo(v).m_worldTransform.getBasis()[1][getRightAxis()],
                               getWheelInfo(v).m_worldTransform.getBasis()[2][getRightAxis()]);

    debugDrawer->drawLine(wheelPosWS, wheelPosWS + axle, wheelColor);
    debugDrawer->drawLine(wheelPosWS, getWheelInfo(v).m_raycastInfo.m_contactPointWS, wheelColor);
    // Draw suspension
    debugDrawer->drawLine(wheelPosWS, getWheelInfo(v).m_raycastInfo.m_hardPointWS, {1, 0, 0});
  }
}

void RaycastVehicle::applyAerodynamics(btScalar step)
{
  (void)step;

  const btVector3& velocity = m_chassisBody->getLinearVelocity();
  const btScalar speed2 = velocity.length2();

  if (speed2 < 0.1) return;

  btVector3 vel_dir = velocity;
  vel_dir.normalize();

  constexpr btScalar kAirDensity = 1.225f;
  constexpr btScalar kAirDragCoef = 1.f;
  constexpr btScalar kFrontalArea = 1.4f;

  constexpr btScalar kCommonCoef = 0.5f * kFrontalArea * kAirDensity;

  btVector3 air_drag_force = -kAirDragCoef * kCommonCoef * speed2 * vel_dir;

  constexpr btScalar kAirLiftCoef = kAirDragCoef * 2.5f;

  btVector3 down_dir = -m_chassisBody->getWorldTransform().getBasis().getColumn(1);

  assert(down_dir.length() < 1.01 && down_dir.length() > 0.99);

  btVector3 air_down_force = kAirLiftCoef * kCommonCoef * speed2 * down_dir;

  // I dont know why but when i use applyCenteralForce
  // it behaves incorrect on debug build
  m_chassisBody->applyImpulse(step * air_down_force, {0., 0., 0.});
  m_chassisBody->applyImpulse(step * air_drag_force, {0., 0., 0.});
}

void RaycastVehicle::updateTyres(btScalar step)
{
  for (int i = 0; i < this->getNumWheels(); i++)
  {
    WheelInfo& wheel = getWheelInfo(i);
    btScalar wearRatePerMin = getTyreWearRatePerMin(wheel.m_tyreInfo.m_type);

    wheel.m_tyreInfo.m_health -= wearRatePerMin * step / 60.;
    if (wheel.m_tyreInfo.m_health < 0.f) wheel.m_tyreInfo.m_health = 0.f;

    //// Calcuate slip ratio
    //{
    //  wheel.m_wheelAngularSpeed=wheel.m_deltaRotation/step;
    //  btScalar wheelLinearSpeed=
    //    wheel.m_wheelAngularSpeed*wheel.m_wheelsRadius;
    //  btScalar vehicleSpeed=
    //    m_chassisBody->getLinearVelocity().length();

    //  if(vehicleSpeed>0.1f)
    //    wheel.m_slipRatio=1.f-wheelLinearSpeed/vehicleSpeed;
    //  else
    //    wheel.m_slipRatio=0.f;
    //}

    //// update temperature
    //{
    //  btScalar& tempCel=wheel.m_tyreInfo.m_tempCelsius;

    //  btScalar tempIncrease=0;
    //  tempIncrease+=
    //    wheel.m_slipRatio*WheelInfo::TyreInfo::s_slipRatioTempConstant;
    //  tempIncrease+=
    //    wheel.m_wheelAngularSpeed*
    //    WheelInfo::TyreInfo::s_angularSpeedTempConstant;

    //  // TODO add wetness of ground
    //  btScalar tempDecrease=0;
    //  tempDecrease+=
    //    wheel.m_wheelAngularSpeed*
    //    WheelInfo::TyreInfo::s_angularSpeedTempCoolingConst;

    //  tempCel+=tempIncrease*step;
    //  tempCel-=tempDecrease*step;
    //}

    //// update health
    //{
    //  btScalar wearRatePerMin=getTyreWearRatePerMin(wheel.m_tyreInfo.m_type);

    //  wheel.m_tyreInfo.m_health-=wearRatePerMin*step/60.f;
    //  if(wheel.m_tyreInfo.m_health<0.f)
    //    wheel.m_tyreInfo.m_health=0.f;
    //}
  }
}

void RaycastVehicle::updateFrictionBasedOnSurface(btScalar step)
{
  (void)step;
  // WARNING
  // Unsafe access sometimes via nullptr
  for (int i = 0; i < this->getNumWheels(); i++)
  {
    WheelInfo& wheel = this->getWheelInfo(i);
    VehicleRaycaster::VehicleRaycasterResult result;
    btRigidBody* p_ground = wheel.m_raycastInfo.m_groundObject;

    if (!p_ground) continue;

    if (!p_ground->isStaticObject()) continue;

    if (!p_ground->getUserPointer()) continue;

    // Here it might be unsave
    BulletUserData* user_data = (BulletUserData*)(p_ground->getUserPointer());

    if (user_data->getType() != BulletUserData::Type::Ground) continue;

    Ground::SurfaceInfo* surface_info = (Ground::SurfaceInfo*)(user_data);

    wheel.m_rollInfluence = surface_info->rolling_resistance;
  }
}

btScalar RaycastVehicle::getTyreWearRatePerMin(WheelInfo::TyreType type)
{
  switch (type)
  {
    case WheelInfo::TyreType::Hard:
      return WheelInfo::TyreInfo::s_hardWearRatePerMin;
    case WheelInfo::TyreType::Soft:
      return WheelInfo::TyreInfo::s_softWearRatePerMin;
    case WheelInfo::TyreType::Wet:
      return WheelInfo::TyreInfo::s_wetWearRatePerMin;
    default:
      btAssert(false && "Unknown TyreType");
      return WheelInfo::TyreInfo::s_softWearRatePerMin;
  }
}

btScalar RaycastVehicle::updateDriveParts(btScalar step)
{
  (void)step;

  auto drive_torque = 0.0f;
  if (m_throttle == 0)
  {
    drive_torque = -70 * m_gearbox.GetCurrentRatio() * Gearbox::kDifferentialRatio * kTransmissionEfficiency;
  }
  else if (!m_engine.is_revLimiter_active)
  {
    drive_torque = m_engine.GetMaxTorque() * m_throttle * m_gearbox.GetCurrentRatio() * Gearbox::kDifferentialRatio *
                   kTransmissionEfficiency;
  }
  return drive_torque;
}

bool RaycastVehicle::setGearUp()
{
  auto new_rpms = -1.f;
  auto current_gear = static_cast<uint8_t>(m_gearbox.current_gear);
  auto is_neutral = current_gear == static_cast<uint8_t>(Gear::Neutral);

  if (is_neutral)
  {
    auto avg_ang_speed = (m_wheelsInfo[static_cast<uint8_t>(WheelPosition::RearLeft)].m_angSpeed +
                          m_wheelsInfo[static_cast<uint8_t>(WheelPosition::RearRight)].m_angSpeed) /
                         2;
    new_rpms = avg_ang_speed * m_gearbox.kGearRatios[current_gear + 1] * Gearbox::kDifferentialRatio *
               (60.0f / (2.0f * 3.14159f));
  }
  else if (current_gear == static_cast<uint8_t>(Gear::Reverse))
  {
    if (m_wheelsInfo[static_cast<uint8_t>(WheelPosition::RearLeft)].m_angSpeed == 0 &&
        m_wheelsInfo[static_cast<uint8_t>(WheelPosition::RearRight)].m_angSpeed == 0)
    {
      m_gearbox.current_gear = static_cast<Gear>(current_gear + 1);
      return true;
    }
  }
  else if (current_gear != (std::size(m_gearbox.kGearRatios) - 1))
  {
    new_rpms = m_engine.rpm * (m_gearbox.kGearRatios[current_gear + 1] / m_gearbox.GetCurrentRatio());
  }

  if (new_rpms >= 3000 || is_neutral)
  {
    m_engine.SetNewRPM(new_rpms);
    m_gearbox.current_gear = static_cast<Gear>(current_gear + 1);
    return true;
  }
  return false;
}

bool RaycastVehicle::setGearDown()
{
  auto current_gear = static_cast<uint8_t>(m_gearbox.current_gear);
  if (current_gear == static_cast<uint8_t>(Gear::Neutral))
  {
    if (m_wheelsInfo[static_cast<uint8_t>(WheelPosition::RearLeft)].m_angSpeed == 0 &&
        m_wheelsInfo[static_cast<uint8_t>(WheelPosition::RearRight)].m_angSpeed == 0)
    {
      m_gearbox.current_gear = static_cast<Gear>(current_gear - 1);
      return true;
    }
  }
  else if (current_gear != 0)
  {
    auto new_rpms = m_engine.rpm * (m_gearbox.kGearRatios[current_gear - 1] / m_gearbox.GetCurrentRatio());
    if (new_rpms < 15000)
    {
      m_engine.SetNewRPM(new_rpms);
      m_gearbox.current_gear = static_cast<Gear>(current_gear - 1);
      return true;
    }
  }
  return false;
}

btScalar RaycastVehicle::getWheelLongSpeed(WheelInfo& wheel) const
{
  btVector3 contactNormal;
  if (wheel.m_raycastInfo.m_isInContact)
  {
    contactNormal = wheel.m_raycastInfo.m_contactNormalWS;
  }
  else
  {
    // for now this should be unreachable, because long_dir is not used if wheel is not touching ground. It stayed for
    // possible future uses.
    contactNormal = m_chassisBody->getWorldTransform().getBasis().getColumn(1);  // Os Y auta
  }
  btVector3 axleDir = -wheel.m_worldTransform.getBasis().getColumn(m_indexRightAxis);

  btVector3 forward_dir = contactNormal.cross(axleDir);
  forward_dir = forward_dir.normalize();

  // std::cout << "dir_X:  " << forward_dir.getX() << "\t";
  // std::cout << "dir_Y:  " << forward_dir.getY() << "\t";
  // std::cout << "dir_Z:  " << forward_dir.getZ() << "\t";

  auto vel_at_contact = getWheelContactVel(wheel);

  // here should be consideration of movement of ground if it is not static object (more realistic if one car will
  // speeding on another)

  return vel_at_contact.dot(forward_dir);
}

btScalar RaycastVehicle::getWheelLatSpeed(WheelInfo& wheel) const
{
  btVector3 axleDir = -wheel.m_worldTransform.getBasis().getColumn(m_indexRightAxis);
  // std::cout << "axle_dir_X: " << axleDir.getX() << "\t";
  // std::cout << "axle_dir_Y: " << axleDir.getY() << "\t";
  // std::cout << "axle_dir_Z: " << axleDir.getZ() << "\n";
  auto vel_at_contact = getWheelContactVel(wheel);
  return vel_at_contact.dot(axleDir);
}

btVector3 RaycastVehicle::getWheelContactVel(WheelInfo& wheel) const
{
  auto chasis = getRigidBody();
  auto contact_point = wheel.m_raycastInfo.m_contactPointWS;
  btVector3 rel_pos = contact_point - chasis->getCenterOfMassPosition();
  return chasis->getVelocityInLocalPoint(rel_pos);
}

}  // namespace boink
