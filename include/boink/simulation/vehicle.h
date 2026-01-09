#pragma once

#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>
#include <BulletDynamics/Vehicle/btVehicleRaycaster.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btMotionState.h>
#include <memory>
#include <vector>

namespace boink
{
  class Vehicle
  {
  public:
    Vehicle(
        std::string_view filename, 
        btScalar mass,
        std::shared_ptr<btDynamicsWorld> world);
    Vehicle(const Vehicle&)=delete;
    Vehicle(Vehicle&&)=default;

    Vehicle& operator=(const Vehicle&)=delete;
    Vehicle& operator=(Vehicle&&)=default;

    ~Vehicle() noexcept;
  private:
    std::unique_ptr<btCollisionShape> createCollisonShape(
        const btVector3& scale,
        const std::vector<btVector3>& vertices);
    std::unique_ptr<btRigidBody> createRigidbody(
        btScalar mass);
  private:
    static constexpr std::string_view BODY_NAME="Cylinder.002";
    static constexpr std::string_view REAR_LEFT_WHEEL_NAME="Cylinder.005";
    static constexpr std::string_view REAR_RIGHT_WHEEL_NAME="Cylinder.004";
    static constexpr std::string_view FRONT_LEFT_WHEEL_NAME="Cylinder.003";
    static constexpr std::string_view FRONT_RIGHT_WHEEL_NAME="Cylinder.007";
  private:
    std::shared_ptr<btDynamicsWorld> world_;

    std::unique_ptr<btCollisionShape> collision_shape_;
    std::unique_ptr<btMotionState> motion_state_;
    std::unique_ptr<btRigidBody> rigidbody_;
    std::unique_ptr<btVehicleRaycaster> raycaster_;
    std::unique_ptr<btRaycastVehicle> vehicle_;

    btRaycastVehicle::btVehicleTuning tuning_;
  };
}
