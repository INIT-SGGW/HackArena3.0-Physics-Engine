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

#include "boink/simulators/track/ground.h"
#include "boink/simulators/vehicle/physics/helpers/car-drive-parts/engine.h"
#include "boink/simulators/vehicle/physics/helpers/car-drive-parts/gearbox.h"
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
  // constructor to create a car from an existing rigidbody
  RaycastVehicle(btRigidBody* chassis, VehicleRaycaster* raycaster);
  virtual ~RaycastVehicle() = default;

  virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step);

  void debugDraw(btIDebugDraw* debugDrawer);
  void enableDraw(bool enable = true) { m_drawEnable = enable; }

  const btTransform& getChassisWorldTransform() const;
  const btTransform& getWheelTransformWS(int wheelIndex) const;
  void updateWheelTransform(int wheelIndex, bool interpolatedTransform = true);
  void updateWheelTransformsWS(WheelInfo& wheel, bool interpolatedTransform = true);

  WheelInfo& addWheel(const btVector3& connectionPointCS0, const btVector3& wheelDirectionCS0,
                      const btVector3& wheelAxleCS, btScalar suspensionRestLength, btScalar wheelRadius,
                      const VehicleTuning& tuning, bool isFrontWheel, WheelInfo::TyreType tyreType);

  inline int getNumWheels() const { return int(m_wheelsInfo.size()); }
  const WheelInfo& getWheelInfo(int index) const;
  WheelInfo& getWheelInfo(int index);

  btScalar getSteeringValue(int wheel) const;
  void setSteeringValue(btScalar steering, int wheel);
  void setBrake(btScalar brake);

  btScalar rayCast(WheelInfo& wheel);

  void resetSuspension();
  virtual void updateSuspension(btScalar deltaTime);
  virtual void updateFriction(btScalar timeStep);
  /// <summary>
  /// Oblicza dzialanie czesci napedowych (silnik, skrzynia) i zwraca moment obrotowy wygenerowany przez te czesci,
  /// ktory trzeba dyferencjalem rozdzielic na kola napedowe
  /// Wazne zeby gdzies po tej funkcji zaktualizowac obroty silnika w zaleznosci od predkosci kol, zeby w nastepnej
  /// klatce jak sie wywola ta metoda to zeby liczylo dla zaktualizowanych obrotow silnika.
  /// </summary>
  /// <param name="deltaTime"></param>
  /// <returns>Wygenerowany moment obrotowy.</returns>
  btScalar updateDriveParts(btScalar deltaTime);

  /// <summary>
  /// Sets gear up.
  /// </summary>
  /// <returns>Whether gear was really upped.</returns>
  bool setGearUp();

  /// <summary>
  /// Sets gear down.
  /// </summary>
  /// <returns>Whether gear was really downed.</returns>
  bool setGearDown();

  /// <summary>
  /// Zwraca globalna szybkosc kola w kierunku, w ktorym kolo jest zwrocone.
  /// </summary>
  btScalar getWheelLongSpeed(WheelInfo& wheel) const;

  /// <summary>
  /// Zwraca globalna szybkosc kola w kierunku poprzecznym kola (na kierunku osi kola).
  /// </summary>
  btScalar getWheelLatSpeed(WheelInfo& wheel) const;

  /// <summary>
  /// Zwraca globalny wektor predkosci kola w punkcie kontaktu kola z podlozem.
  /// </summary>
  btVector3 getWheelContactVel(WheelInfo& wheel) const;

  /// <returns>Information about surface under the wheel or nullptr if there is no surface under the wheel.</returns>
  const Ground::SurfaceInfo* getSurfInfo(WheelInfo& wheel) const;

  inline btRigidBody* getRigidBody() { return m_chassisBody; }
  const btRigidBody* getRigidBody() const { return m_chassisBody; }

  inline int getRightAxis() const { return m_indexRightAxis; }
  inline int getUpAxis() const { return m_indexUpAxis; }
  inline int getForwardAxis() const { return m_indexForwardAxis; }

  btScalar getEngineRPM() const;
  int getCurrentGear() const;
  btScalar getCurrentSpeedKmHour() const { return m_currentVehicleSpeedKmHour; }

  void setPitchControl(btScalar pitch) { m_pitchControl = pitch; }

  virtual void setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex);

  int getUserConstraintType() const { return m_userConstraintType; }
  void setUserConstraintType(int userConstraintType) { m_userConstraintType = userConstraintType; }

  void setUserConstraintId(int uid) { m_userConstraintId = uid; }
  int getUserConstraintId() const { return m_userConstraintId; }

  btScalar m_throttle;
  btScalar m_steeringValue;

 private:
  void applyAerodynamics(btScalar step);

 private:
  btAlignedObjectArray<btVector3> m_forwardWS;
  btAlignedObjectArray<btVector3> m_axle;
  btAlignedObjectArray<btScalar> m_forwardImpulse;
  btAlignedObjectArray<btScalar> m_sideImpulse;
  int m_userConstraintType;
  int m_userConstraintId;

  VehicleRaycaster* m_vehicleRaycaster;
  btScalar m_pitchControl;

  btScalar m_currentVehicleSpeedKmHour;

  btRigidBody* m_chassisBody;
  Engine m_engine;
  Gearbox m_gearbox;

  int m_indexRightAxis;
  int m_indexUpAxis;
  int m_indexForwardAxis;

  btAlignedObjectArray<WheelInfo> m_wheelsInfo;

  bool m_drawEnable = true;

  static constexpr float kAirTemperature = 20.f;  // [Celsius]
  static constexpr float kTransmissionEfficiency = 0.7f;
  static constexpr float kSmoothingTractionForceFactor = 0.35f;
  static constexpr float kBrakeTorque = 3300.0f;  // [Nm]
  static constexpr float kSlipRatioPeak = 0.1f;   // slip ratio with maximum longitudinal grip
  static inline const Curve kSlipRatioToGrip =
      Curve({0.000, 1.100, 1.600, 1.500, 1.350, 1.250, 1.200, 1.150, 1.120, 1.100, 1.080,
             1.060, 1.050, 1.040, 1.030, 1.020, 1.010, 1.000, 1.000, 1.000, 1.000},
            0.05f, 0.0f);
  static constexpr float kSlipAnglePeak = 0.08f;  // slip angle with maximum lateral grip in radians (5.7 degrees)
  static inline const Curve kSlipAngleToGrip = Curve({0.00, 0.50, 0.95, 1.35, 1.55, 1.50, 1.35, 1.20, 1.12, 1.08, 1.06,
                                                      1.05, 1.05, 1.05, 1.05, 1.05, 1.05, 1.05, 1.05, 1.05, 1.05},
                                                     0.02f, 0.0f);
  static inline const Curve kWearToGripCoeff =
      Curve({1.00, 1.00, 0.99, 0.98, 0.97, 0.96, 0.94, 0.91, 0.88, 0.86, 0.85}, -0.1f, 1.f);
};
}  // namespace boink
