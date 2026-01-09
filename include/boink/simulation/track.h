#pragma once

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include <memory>
#include <string_view>

namespace boink
{
  class Track
  {
  public:
    Track(std::string_view filename,
      std::shared_ptr<btDiscreteDynamicsWorld> world);
    Track(const Track&)=delete;
    Track(Track&&)=default;

    Track& operator=(const Track&)=delete;
    Track& operator=(Track&&)=default;

    ~Track() noexcept;
  private:
    std::unique_ptr<btTriangleMesh> mesh_;
    std::unique_ptr<btBvhTriangleMeshShape> collision_shape_;
    std::unique_ptr<btDefaultMotionState> motion_state_;
    std::unique_ptr<btRigidBody> rigidbody_;

    std::shared_ptr<btDiscreteDynamicsWorld> world_;
  };
}
