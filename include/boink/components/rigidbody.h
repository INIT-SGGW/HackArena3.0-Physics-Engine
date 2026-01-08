#pragma once

#include "bullet/LinearMath/btVector3.h"
#include "bullet/LinearMath/btScalar.h"
//#include "bullet/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/BulletDynamics/Dynamics/btRigidBody.h"

#include <LinearMath/btDefaultMotionState.h>
#include <memory>

namespace boink
{
  struct Rigidbody
  {
    btScalar mass;
    btVector3 local_inertia;

    std::shared_ptr<btRigidBody> rigidbody;
  };
}
