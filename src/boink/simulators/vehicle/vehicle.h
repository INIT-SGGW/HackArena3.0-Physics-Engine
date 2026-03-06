#pragma once

#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btMotionState.h>

#include <memory>

#include "boink/simulators/simulator.h"
#include "boink/simulators/track/track.h"
#include "boink/simulators/vehicle/physics/raycast_vehicle.h"
#include "boink/simulators/vehicle/physics/wheel_info.h"
#include "boink/simulators/vehicle/vehicle_mesh.h"
#include "boink/simulators/vehicle/wheel_position.h"

namespace boink
{
class VehicleGui;
class Vehicle : public Simulator
{
  friend class VehicleGui;

 public:
  struct CreationInfo
  {
    std::shared_ptr<const VehicleMesh> mesh;
    btScalar mass;
    btScalar wheel_radius;
    btScalar suspension_rest_length;
    btScalar max_steer_angle;
    btVector3 center_of_mass;
    RaycastVehicle::VehicleTuning tuning;
    WheelInfo::TyreType tyre_type;
  };

  enum class TurnDirection
  {
    Left,
    Right
  };

 public:
  Vehicle(const CreationInfo& create_info, std::shared_ptr<const Track> track, std::shared_ptr<btDynamicsWorld> world);
  Vehicle(const Vehicle&) = delete;
  Vehicle(Vehicle&&) noexcept = default;

  Vehicle& operator=(const Vehicle&) = delete;
  Vehicle& operator=(Vehicle&&) noexcept = default;

  ~Vehicle() noexcept;

  void update(btScalar dt) override;
  void updateRender(Renderer* renderer) override;
  std::shared_ptr<piksel::GuiObject> getGui() override;

  void setPosition(const btVector3& position);

  int getLapsCompleted() const { return laps_completed_; }
  btScalar getCurrentLapDistanceCovered() const { return curr_lap_dist_point_; }

  btTransform getWorldTransform() const;
  btTransform getChassisWorldTransform() const;

  const btTransform& getWheelWorldTransform(WheelPosition wheel_pos) const;
  btScalar getWheelAngularSpeed(WheelPosition wheel_pos) const;
  const btTransform& getCenterOfMassTransform() const;

  btScalar getSpeed() const;
  btScalar getMass() const;
  btVector3 getCenterOfMassCS() const;

  btScalar getTyreHealth(WheelPosition pos) const;
  WheelInfo::TyreType getTyreType(WheelPosition pos) const;
  btScalar getTyreTempCelsius(WheelPosition pos) const;

  void setTuning(const RaycastVehicle::VehicleTuning& tuning);
  const RaycastVehicle::VehicleTuning& getTuning() const;

  // Value from [0,1]
  void setSteering(btScalar value, TurnDirection dir);
  void setEngineForce(btScalar force);
  void setBrake(btScalar brake);
  void setGearUp();
  void setGearDown();

 private:
  void correctCOM();
  std::unique_ptr<btCompoundShape> createCollisonShape(const std::vector<btVector3>& vertices,
                                                       const btVector3& center_of_mass);
  std::unique_ptr<btRigidBody> createRigidbody(btScalar mass);

 private:
  std::shared_ptr<const VehicleMesh> mesh_;
  std::shared_ptr<btDynamicsWorld> world_;

  std::unique_ptr<btCompoundShape> collision_shape_;
  std::unique_ptr<btMotionState> motion_state_;
  std::unique_ptr<btRigidBody> rigidbody_;
  std::unique_ptr<VehicleRaycaster> raycaster_;
  std::unique_ptr<RaycastVehicle> vehicle_;

  std::shared_ptr<const Track> track_;

  btVector3 center_of_mass_;
  btScalar max_steer_angle_;
  RaycastVehicle::VehicleTuning tuning_;

  int laps_completed_ = 0;
  btScalar curr_lap_dist_point_ = 0;

  std::shared_ptr<VehicleGui> gui_;
};
}  // namespace boink
