#pragma once

#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

#include "boink/bullet_user_data.h"
#include "boink/simulators/vehicle/vehicle.h"

namespace boink
{
  struct RayResultCallback : public btCollisionWorld::ClosestRayResultCallback
  {
    RayResultCallback(
        const btVector3& rayFromWorld, const btVector3& rayToWorld, bool isGhosted)
        : btCollisionWorld::ClosestRayResultCallback(rayFromWorld, rayToWorld),
        m_isGhosted(isGhosted)
    {
    }

    virtual btScalar addSingleResult(
        btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
    {
      const btCollisionObject* obj = rayResult.m_collisionObject;
      const btRigidBody* body = btRigidBody::upcast(obj);

      if(m_isGhosted&&
          obj->getBroadphaseHandle()->m_collisionFilterGroup==Vehicle::GROUP_MASK)
        return m_closestHitFraction;

      if (body)
      {
        BulletUserData* user_data=
            reinterpret_cast<BulletUserData*>(body->getUserPointer());

        if(user_data)
        {
          Vehicle::UserData* vehicle_data = 
              reinterpret_cast<Vehicle::UserData*>(user_data);

          if (vehicle_data->getType()==BulletUserData::Type::Vehicle &&
              vehicle_data->ghost_info->enabled)
          {
            // Return 1.0 (or current m_closestHitFraction) to tell Bullet:
            // "Ignore this hit, and keep searching the rest of the ray."
            return m_closestHitFraction; 
          }
        }
      }

      return btCollisionWorld::ClosestRayResultCallback::addSingleResult(
          rayResult, normalInWorldSpace);
    }

  private:
    bool m_isGhosted;
  };
}
