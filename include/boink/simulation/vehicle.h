#pragma once

#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>
#include <BulletDynamics/Vehicle/btVehicleRaycaster.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btMotionState.h>

#include "boink/simulation/vehicle_mesh.h"
#include "boink/simulation/wheel_position.h"

#include <memory>

namespace boink
{
  class Vehicle
  {
  public:
    struct CreationInfo
    {
      std::shared_ptr<VehicleMesh> mesh;
      btScalar mass;
      btScalar wheel_radius;
      btScalar suspension_rest_length;
      btVector3 center_of_mass;
    };
  public:
    Vehicle(
        const CreationInfo& create_info,
        std::shared_ptr<btDynamicsWorld> world);
    Vehicle(const Vehicle&)=delete;
    Vehicle(Vehicle&&)=default;

    Vehicle& operator=(const Vehicle&)=delete;
    Vehicle& operator=(Vehicle&&)=default;

    ~Vehicle() noexcept;

    void setPosition(const btVector3& position);

    btTransform getWorldTransform() const;
    btTransform getChassisWorldTransform() const;
    const btTransform& getWheelWorldTransform(WheelPosition wheel_pos) const;
    const btTransform& getCenterOfMassTransform() const;
  private:
    std::shared_ptr<btCollisionShape> createCollisonShape(
        const std::vector<btVector3>& vertices,
        const btVector3& center_of_mass);
    std::unique_ptr<btRigidBody> createRigidbody(
        btScalar mass);
  private:
    std::shared_ptr<VehicleMesh> mesh_;
    std::shared_ptr<btDynamicsWorld> world_;

    std::shared_ptr<btCollisionShape> collision_shape_;
    std::unique_ptr<btMotionState> motion_state_;
    std::unique_ptr<btRigidBody> rigidbody_;
    std::unique_ptr<btVehicleRaycaster> raycaster_;
    std::unique_ptr<btRaycastVehicle> vehicle_;

    btRaycastVehicle::btVehicleTuning tuning_;

    btVector3 center_of_mass_;
  };
}
