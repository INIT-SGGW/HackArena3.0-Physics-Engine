#pragma once

#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

#include "boink/bullet_user_data.h"
#include "boink/simulators/vehicle/vehicle.h"
#include "boink/collision_group.h"
namespace boink
{
  struct RayResultCallback : public btCollisionWorld::ClosestRayResultCallback
  {
    RayResultCallback(
        const btVector3& rayFromWorld, 
        const btVector3& rayToWorld, 
        bool isGhosted,
        const btCollisionObject* chassis) // 1. Add chassis here!
        : btCollisionWorld::ClosestRayResultCallback(rayFromWorld, rayToWorld),
        m_isGhosted(isGhosted),
        m_chassis(chassis)
    {
    }

    virtual btScalar addSingleResult(
        btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace) override
    {
      const btCollisionObject* obj = rayResult.m_collisionObject;
      const btRigidBody* body = btRigidBody::upcast(obj);

      // FIX 1: Prevent the car from hitting its own body!
      if (obj == m_chassis) 
        return m_closestHitFraction; 

      // FIX 2: Use bitwise & instead of == just in case groups are combined
      if(m_isGhosted && 
          (obj->getBroadphaseHandle()->m_collisionFilterGroup & 
           Collision::Group::Vehicle))
        return m_closestHitFraction;

      if (body)
      {
        BulletUserData* user_data=
            reinterpret_cast<BulletUserData*>(body->getUserPointer());

        if(user_data && user_data->getType()==BulletUserData::Type::Vehicle)
        {
          Vehicle::UserData* vehicle_data = 
              reinterpret_cast<Vehicle::UserData*>(user_data);

          Vehicle::UserData* my_vehicle_data = 
              reinterpret_cast<Vehicle::UserData*>(m_chassis->getUserPointer());


          if (vehicle_data->ghost_info->enabled)
            return m_closestHitFraction; 
          else if (my_vehicle_data->ghost_info->overlap_vehicles.find(
                const_cast<btCollisionObject*>(obj)) 
              != my_vehicle_data->ghost_info->overlap_vehicles.end())
          {
            return m_closestHitFraction;
          }
        }
      }

      return btCollisionWorld::ClosestRayResultCallback::addSingleResult(
          rayResult, normalInWorldSpace);
    }

  private:
    bool m_isGhosted;
    const btCollisionObject* m_chassis;
  };
}
