#pragma once

#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"

#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"

#include "LinearMath/btIDebugDraw.h"

#include "boink/components/rigidbody.h"

#include <memory>
#include <vector>

namespace boink
{
  class Simulation
  {
  public:
    Simulation();
    ~Simulation() noexcept;

    void addGround(const btVector3& dims, const btVector3& pos);
    void addSphere(btScalar radius, const btVector3& pos);
    Rigidbody createCarRigidbody(
        const std::vector<btVector3>& vertices, 
        const btTransform& trans,
        btScalar mass);
    void step(double dt);
    void registerDebugDrawer(btIDebugDraw* dbg);
  public:
    static constexpr double GRAVITATIONAL_ACCELERATION=10.;
  private:
    std::unique_ptr<btDefaultCollisionConfiguration> collision_configuration;
    std::unique_ptr<btCollisionDispatcher> dispatcher;
    std::unique_ptr<btDbvtBroadphase> overlapping_pair_cache;
    std::unique_ptr<btSequentialImpulseConstraintSolver> solver;

    std::unique_ptr<btDiscreteDynamicsWorld> dynamics_world;
    btAlignedObjectArray<std::shared_ptr<btCollisionShape>> collision_shapes;
  };
}
