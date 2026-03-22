#pragma once

#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include "boink/bullet_user_data.h"
#include "boink/simulators/vehicle/vehicle.h"

namespace boink
{
  class CustomCollisionDispatcher : public btCollisionDispatcher 
  {
  public:
    CustomCollisionDispatcher(btCollisionConfiguration* collisionConfiguration) 
        : btCollisionDispatcher(collisionConfiguration) {}

    virtual bool needsCollision(
        const btCollisionObject* body0, const btCollisionObject* body1) override 
    {
      if (body0->isStaticOrKinematicObject() && body1->isStaticOrKinematicObject())
        return false;

      if (body0->getUserPointer() != nullptr && body1->getUserPointer() != nullptr)
      {
        BulletUserData* ud0 = reinterpret_cast<BulletUserData*>(body0->getUserPointer());
        BulletUserData* ud1 = reinterpret_cast<BulletUserData*>(body1->getUserPointer());

        if (ud0->getType() == BulletUserData::Type::Vehicle && 
            ud1->getType() == BulletUserData::Type::Vehicle)
        {
          Vehicle::UserData* v_ud0 = reinterpret_cast<Vehicle::UserData*>(ud0);
          Vehicle::UserData* v_ud1 = reinterpret_cast<Vehicle::UserData*>(ud1);

          // Jeśli któryś ma ghost_info->enabled, w TEJ klatce odrzucamy kolizję!
          if (v_ud0->ghost_info->enabled || v_ud1->ghost_info->enabled)
            return false; 

          if (v_ud0->ghost_info->overlap_vehicles.find(
                const_cast<btCollisionObject*>(body1)) 
              != v_ud0->ghost_info->overlap_vehicles.end())
            return false;
        }
      }

      return btCollisionDispatcher::needsCollision(body0, body1);
    }
  };
}
