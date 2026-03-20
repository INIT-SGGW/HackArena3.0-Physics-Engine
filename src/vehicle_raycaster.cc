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
#include "boink/simulators/vehicle/physics/vehicle_raycaster.h"

#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

#include "boink/simulators/vehicle/physics/ray_result_callback.h"

namespace boink
{
  VehicleRaycaster::VehicleRaycaster(btDynamicsWorld* world,btRigidBody* chassis)
    : m_dynamicsWorld(world),m_chassis(chassis)
  {
  }

  btRigidBody* VehicleRaycaster::castRay(
        const btVector3& from, 
        const btVector3& to, 
        VehicleRaycasterResult& result)
  {
    Vehicle::UserData* user_data=
      reinterpret_cast<Vehicle::UserData*>(m_chassis->getUserPointer());
    btAssert(user_data!=nullptr);

    RayResultCallback rayCallback(from, to,user_data->ghost_info->enabled);
    rayCallback.m_collisionFilterGroup=Collision::Group::Vehicle;
    rayCallback.m_collisionFilterMask=Collision::Group::All;
    
    m_dynamicsWorld->rayTest(from, to, rayCallback);

    if (rayCallback.hasHit())
    {
      const btRigidBody* body = 
        btRigidBody::upcast(rayCallback.m_collisionObject);

      if (body && body->hasContactResponse())
      {
        result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
        result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
        result.m_hitNormalInWorld.normalize();
        result.m_distFraction = rayCallback.m_closestHitFraction;

        // TODO
        // consider making return type const
        // (did that way cause in src it was done this way)
        return const_cast<btRigidBody*>(body);
      }
    }
    return nullptr;
  }
}
