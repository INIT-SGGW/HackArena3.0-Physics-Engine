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

#include "boink/assert.h"
#include "boink/bullet_user_data.h"
#include "boink/constants.h"
#include "boink/simulators/track/ground.h"
#include "boink/simulators/vehicle/physics/helpers/scalarLerp.h"
#include "boink/simulators/vehicle/physics/vehicle_raycaster.h"
#include "boink/simulators/vehicle/physics/wheel_info.h"
#include "boink/simulators/vehicle/wheel_position.h"

#define ROLLING_INFLUENCE_FIX

btRigidBody& btActionInterface::getFixedBody()
{
  static btRigidBody s_fixed(0, 0, 0);

  static boink::Ground::SurfaceInfo fixed_surf = {0.7f, 0.0f, 0.3f, 0.f, boink::Ground::Type::Asphalt};
  static boink::Ground::UserData fixed_user_data(&fixed_surf);
  s_fixed.setUserPointer(reinterpret_cast<boink::BulletUserData*>(&fixed_user_data));
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
  m_brakeBias = btScalar(0.6f);
  m_brake = btScalar(0.f);
  m_diffSetting = btScalar(0.f);
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
  BOINK_ASSERT(wheelIndex < getNumWheels());

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
  BOINK_ASSERT((index >= 0) && (index < getNumWheels()));

  return m_wheelsInfo[index];
}

WheelInfo& RaycastVehicle::getWheelInfo(int index)
{
  BOINK_ASSERT((index >= 0) && (index < getNumWheels()));

  return m_wheelsInfo[index];
}

btScalar RaycastVehicle::getEngineRPM() const
{
  if (m_engine.m_is_on_idle)
    return m_engine.GetIdleRPM();
  else
    return m_engine.rpm;
}

int RaycastVehicle::getCurrentGear() const { return static_cast<int>(m_gearbox.current_gear); }

btScalar RaycastVehicle::getSteeringValue(int wheel) const { return getWheelInfo(wheel).m_steering; }

void RaycastVehicle::setSteeringValue(btScalar steering, int wheel)
{
  BOINK_ASSERT(wheel >= 0 && wheel < getNumWheels());

  WheelInfo& wheelInfo = getWheelInfo(wheel);
  wheelInfo.m_steering = steering;
}

void RaycastVehicle::setTyreType(WheelInfo::TyreType tyre_type)
{
  for (int i = 0; i < 4; i++)
  {
    WheelInfo& wheelInfo = m_wheelsInfo[i];
    wheelInfo.m_tyreInfo.m_type = tyre_type;
    wheelInfo.m_tyreInfo.m_health = btScalar(1.0);
    wheelInfo.m_tyreInfo.m_tempCelsius = btScalar(20.0);
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

  BOINK_ASSERT(m_vehicleRaycaster);

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

void RaycastVehicle::reset()
{
  resetSuspension();
  m_currentVehicleSpeedKmHour = btScalar(0.f);

  m_throttle = btScalar(0.f);
  m_steeringValue = btScalar(0.f);
  m_brake = btScalar(0.f);

  m_engine.is_revLimiter_active = false;
  m_engine.m_is_on_idle = true;
  m_engine.m_idle_timer = btScalar(0.f);
  m_engine.SetNewRPM(btScalar(0.f));

  m_gearbox.current_gear = Gear::First;

  for (int i = 0; i < getNumWheels(); i++)
  {
    auto& wheel_info = getWheelInfo(i);
    wheel_info.resetWheel();
  }

  m_last_frame_speed = btScalar(0.f);
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

void RaycastVehicle::updateFriction(btScalar timeStep)
{
  // std::cout << "vel_X:  " << getRigidBody()->getLinearVelocity().getX() << "\t";
  // std::cout << "vel_Y:  " << getRigidBody()->getLinearVelocity().getY() << "\t";
  // std::cout << "vel_Z:  " << getRigidBody()->getLinearVelocity().getZ() << "\t";

  int numWheel = getNumWheels();
  if (!numWheel) return;

  m_forwardWS.resize(numWheel);
  m_axle.resize(numWheel);
  m_forwardImpulse.resize(numWheel);
  m_sideImpulse.resize(numWheel);

  // @differential
  auto total_drive_torque = updateDriveParts(timeStep);
  btScalar curr_diff_stiff = scalarLerp(kMinDiff, kMaxDiff, m_diffSetting);

  auto ang_speed_diff =
      m_wheelsInfo[(int)WheelPosition::RearLeft].m_angSpeed - m_wheelsInfo[(int)WheelPosition::RearRight].m_angSpeed;
  auto locking_torque = ang_speed_diff * curr_diff_stiff;
  auto drive_torque_L = (total_drive_torque / 2.f) - locking_torque;
  auto drive_torque_R = (total_drive_torque / 2.f) + locking_torque;

  // std::cout << "drive_torq_L:  " << drive_torque_L << "\n";
  // std::cout << "drive_torq_R:  " << drive_torque_R << "\n";

  // @brake bias
  auto total_brake_torque = (m_gearbox.current_gear != Gear::Reverse) ? -kBrakeTorque : kBrakeTorque;
  total_brake_torque *= m_brake;
  auto front_brake_torque = total_brake_torque * m_brakeBias;
  auto rear_brake_torque = total_brake_torque - front_brake_torque;

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

    // @wheels forces and angular speeds
    btScalar total_torque = 0.f;
    auto traction_torque = 0.f;
    auto drag_torque = 0.f;
    auto drive_torque = 0.f;

    if (groundObject)
    {
      traction_torque = -wheelInfo.m_traction_force * wheelInfo.m_wheelSimRadius;
      drag_torque = wheelInfo.m_drag_long_force * wheelInfo.m_wheelSimRadius;
      if (wheelInfo.m_angSpeed > 0) drag_torque *= -1;
    }

    if (!wheelInfo.m_bIsFrontWheel)
    {
      if (wheel_idx == (int)WheelPosition::RearLeft)
        drive_torque = drive_torque_L;
      else
        drive_torque = drive_torque_R;
      total_torque = drive_torque + traction_torque + drag_torque + rear_brake_torque;
      // std::cout << "rear_brake_torque:  " << rear_brake_torque << "\t";
    }
    else
    {
      total_torque = traction_torque + drag_torque + front_brake_torque;
      // std::cout << "front_brake_torque:  " << front_brake_torque << "\t";
    }

    auto wheel_inertia = wheelInfo.kWheelMass * wheelInfo.m_wheelSimRadius * wheelInfo.kWheelMassDistCoeff;
    btScalar engine_inertia_part = 0.f;
    if (!wheelInfo.m_bIsFrontWheel)
      engine_inertia_part =
          (m_engine.inertia / 2.f) *
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

      auto slip_velocity = wheelInfo.m_angSpeed * wheelInfo.m_wheelSimRadius - speed_SR;

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
        else if (btFabs(speed_SR) < 2.9f)
        {
          slip_ratio = slip_velocity / 2.9f;
        }
        else
        {
          slip_ratio = slip_velocity / btFabs(speed_SR);
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

      btScalar surf_wet = 0.f;
      btScalar surf_grip_coeff = 0.f;
      btScalar surf_drag_coeff = 0.f;

      const auto& tyre_type_info = WheelInfo::kTyresTypesInfo[(int)wheelInfo.m_tyreInfo.m_type];
      auto surf_info = getSurfInfo(wheelInfo);
      if (surf_info)
      {
        surf_wet = surf_info->wetness;
        surf_grip_coeff = surf_info->grip_coeff;
        surf_drag_coeff = surf_info->drag_coeff;
      }

      auto temp_grip_coeff = tyre_type_info.tempToGripCoeff.GetValue(wheelInfo.m_tyreInfo.m_tempCelsius);
      auto wear_grip_coeff = kWearToGripCoeff.GetValue(wheelInfo.m_tyreInfo.m_health);
      auto surf_wet_grip_coeff = scalarLerp(tyre_type_info.baseDryGrip, tyre_type_info.baseWetGrip, surf_wet);
      auto temp_stiff_coeff = tyre_type_info.tempToStiffCoeff.GetValue(wheelInfo.m_tyreInfo.m_tempCelsius);

      // std::cout << "surf_wet_grip_coeff: " << surf_wet_grip_coeff << "\t";
      // std::cout << "temp_grip_coeff: " << temp_grip_coeff << "\t";
      // std::cout << "temp_stiff_coeff: " << temp_stiff_coeff << "\t";
      /*std::cout << "tyre_health: " << wheelInfo.m_tyreInfo.m_health << "\t";
      std::cout << "wear_grip_coeff: " << wear_grip_coeff << "\t";*/

      // this represent that temperature or wear of tyre is not important on other surface than asphalt
      auto total_grip_coeff = (surf_grip_coeff == 1.f)
                                  ? surf_grip_coeff * surf_wet_grip_coeff * temp_grip_coeff * wear_grip_coeff
                                  : surf_grip_coeff * surf_wet_grip_coeff;

      BOINK_ASSERT(temp_stiff_coeff > g_Epsilon);
      auto curr_peak_lat = kSlipAnglePeak / temp_stiff_coeff;
      auto curr_peak_long = kSlipRatioPeak / temp_stiff_coeff;

      BOINK_ASSERT(curr_peak_lat > g_Epsilon);
      BOINK_ASSERT(curr_peak_long > g_Epsilon);
      auto slip_ang_normalized = slip_angle / curr_peak_lat;
      auto slip_ratio_normalized = slip_ratio / curr_peak_long;

      auto slip_vec_len =
          btSqrt(slip_ang_normalized * slip_ang_normalized + slip_ratio_normalized * slip_ratio_normalized);
      wheelInfo.m_slip_vec_length = slip_vec_len;

      // std::cout << "slip_vec_len:  " << slip_vec_len << "\t";

      if (slip_vec_len > g_Epsilon)
      {
        auto slip_ang_scaled = slip_vec_len * kSlipAnglePeak;
        auto slip_ratio_scaled = slip_vec_len * kSlipRatioPeak;

        auto lat_grip_base = kSlipAngleToGrip.GetValue(slip_ang_scaled) * total_grip_coeff;
        auto long_grip_base = kSlipRatioToGrip.GetValue(slip_ratio_scaled) * total_grip_coeff;

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

      // @drag
      auto wheel_vel = getWheelContactVel(wheelInfo);
      auto wheel_speed = wheel_vel.length();
      auto dynamic_drag_coeff = surf_drag_coeff * wheel_speed * 0.3f;
      auto drag_force = dynamic_drag_coeff * wheelInfo.m_wheelsSuspensionForce;
      auto drag_direction = -wheel_vel.normalized();
      auto drag_impulse = drag_direction * drag_force * timeStep;

      // std::cout << "drag_impulse: " << drag_impulse.length() << "\t";
      // std::cout << "wheel_speed: " << wheel_speed << "\t";

      if (wheel_speed > 0.1f)
      {
        auto drag_long_percent = btFabs(long_speed) / wheel_speed;
        wheelInfo.m_drag_long_force = drag_force * drag_long_percent;
      }
      else
        wheelInfo.m_drag_long_force = 0.f;

      // @temperature
      auto lat_power = btFabs(lateral_force * lat_speed);
      auto long_power = btFabs(traction_force * slip_velocity);

      auto slip_power = lat_power + long_power;
      auto rolling_power = 0.1f * wheelInfo.m_wheelsSuspensionForce * wheel_speed;  // temperature from wheel squishing
      auto heat_generated =
          (slip_power + rolling_power) * WheelInfo::TyreInfo::heatingConst * tyre_type_info.heatingFactor * timeStep;
      auto speed_factor = btMax(5.f, wheel_speed);  // minimal value when car is stopped
      auto wet_factor = 1.f + (surf_wet * 4.f);
      auto heat_lost = (wheelInfo.m_tyreInfo.m_tempCelsius - kAirTemperature) * WheelInfo::TyreInfo::coolingConst *
                       speed_factor * wet_factor * timeStep;
      wheelInfo.m_tyreInfo.m_tempCelsius += heat_generated - heat_lost;
      if (wheelInfo.m_tyreInfo.m_tempCelsius > 160.f) wheelInfo.m_tyreInfo.m_tempCelsius = btScalar(160.f);
      if (wheelInfo.m_tyreInfo.m_tempCelsius < 0.f) wheelInfo.m_tyreInfo.m_tempCelsius = btScalar(0.f);

      // std::cout << "wetness: " << surf_wet << "\t";
      // std::cout << "tyre_type: " << (int)wheelInfo.m_tyreInfo.m_type << "\t";
      // std::cout << "heat_gen: " << heat_generated << "\t";
      // std::cout << "temperature: " << wheelInfo.m_tyreInfo.m_tempCelsius << "\t";

      // @wear
      slip_power = lat_power + long_power;
      wheelInfo.m_tyreInfo.m_health -= slip_power * WheelInfo::TyreInfo::wearRate * timeStep;
      if (wheelInfo.m_tyreInfo.m_health < 0.f) wheelInfo.m_tyreInfo.m_health = 0.f;

      // std::cout << "wear: " << wheelInfo.m_tyreInfo.m_health << "\t";

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
      if (drag_impulse.length() != 0.f)
      {
        m_chassisBody->applyImpulse(drag_impulse, rel_pos);
      }
    }
    else
    {
      wheelInfo.m_traction_force = 0.0f;
      wheelInfo.m_drag_long_force = 0.f;
    }
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

  auto car_linear_speed = getRigidBody()->getLinearVelocity().length();

  if (car_linear_speed < 0.1f && m_last_frame_speed >= 0.1f && m_gearbox.current_gear != Gear::First &&
      m_gearbox.current_gear != Gear::Reverse)
  {
    m_gearbox.current_gear = Gear::Neutral;
    m_engine.m_is_on_idle = true;
  }
  m_last_frame_speed = car_linear_speed;

  if (new_rpm < 4000)
    m_engine.m_is_on_idle = true;
  else
  {
    m_engine.m_is_on_idle = false;
    m_engine.m_idle_timer = 0.f;
  }
  if (m_engine.m_is_on_idle) m_engine.UpdateIdleRPMTimer(timeStep);

  if (car_linear_speed < 1.f && m_gearbox.current_gear == Gear::Neutral)
  {
    m_brake = 1;
  }

  // std::cout << "gear:  " << m_gearbox.current_gear << "\t";
  // std::cout << "rpm:  " << m_engine.rpm << "\t";
  //// std::cout << "idle_rpm:  " << m_engine.GetIdleRPM() << "\t";
  // std::cout << "displayed_rpm:  " << getEngineRPM() << "\t";
  // std::cout << "speed: " << getRigidBody()->getLinearVelocity().length() << "\n\n";
  // std::cout << "\n";
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

  BOINK_ASSERT(down_dir.length() < 1.01 && down_dir.length() > 0.99);

  btVector3 air_down_force = kAirLiftCoef * kCommonCoef * speed2 * down_dir;

  // I dont know why but when i use applyCenteralForce
  // it behaves incorrect on debug build
  m_chassisBody->applyImpulse(step * air_down_force, {0., 0., 0.});
  m_chassisBody->applyImpulse(step * air_drag_force, {0., 0., 0.});
}

btScalar RaycastVehicle::updateDriveParts(btScalar step)
{
  (void)step;

  btScalar drive_torque = 0.0f;
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
  btScalar new_rpms = -1.f;
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

  if (new_rpms >= 4000 || is_neutral)
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

const Ground::SurfaceInfo* RaycastVehicle::getSurfInfo(WheelInfo& wheel) const
{
  btRigidBody* p_ground = wheel.m_raycastInfo.m_groundObject;

  if (!p_ground)
  {
    BOINK_ASSERT(
        false &&
        "Wheel is not in contact with ground. This should be unreachable, because this method is called only if "
        "wheel is in contact.");
    return nullptr;
  }

  if (!p_ground->getUserPointer())
  {
    // BOINK_ASSERT(false && "Something is broken with pointers, ask Igor");
    return nullptr;
  }

  BulletUserData* user_data = (BulletUserData*)(p_ground->getUserPointer());

  if (!p_ground->isStaticObject())
  {
    // TODO: Make another surface type like Vehicle in SurfaceInfo, ask Igor for that
    if (user_data->getType() == BulletUserData::Type::Vehicle)
      return nullptr;
    else
      return nullptr;
  }

  if (user_data->getType() != BulletUserData::Type::Ground)
  {
    BOINK_ASSERT(false && "User data is not Type::Ground");
    return nullptr;
  }

  Ground::UserData* user_data_casted = (Ground::UserData*)user_data;
  const Ground::SurfaceInfo* surface_info = user_data_casted->p_surface_info;
  if (surface_info == nullptr)
  {
    BOINK_ASSERT(false && "Ground::SurfaceInfo pointer is null");
    return nullptr;
  }
  auto surface_type = surface_info->type;

  // std::cout << "surface:  " << Ground::toString(surface_type) << "\t";

  if (surface_type < (Ground::Type)0 || surface_type >= Ground::Type::Count)
  {
    BOINK_ASSERT(false && "Invalid Ground::Type");
    return nullptr;
  }
  else
    return surface_info;
}
}  // namespace boink
