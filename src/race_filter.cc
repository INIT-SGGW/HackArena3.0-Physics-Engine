#include "boink/simulation/race_filter.h"

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>

#include "boink/simulators/vehicle/vehicle.h"

namespace boink
{
  bool RaceFilter::needBroadphaseCollision(
      btBroadphaseProxy* proxy0, 
      btBroadphaseProxy* proxy1) const
  {
    btCollisionObject* obj0=static_cast<btCollisionObject*>(proxy0->m_clientObject);
    btCollisionObject* obj1=static_cast<btCollisionObject*>(proxy1->m_clientObject);

    if((proxy0->m_collisionFilterGroup & Vehicle::GROUP_MASK)&&
        (proxy1->m_collisionFilterGroup & Vehicle::GROUP_MASK))
    {
      Vehicle::UserData* data0=
        reinterpret_cast<Vehicle::UserData*>(obj0->getUserPointer());
      Vehicle::UserData* data1=
        reinterpret_cast<Vehicle::UserData*>(obj1->getUserPointer());

      if((data0 && data0->ghost_info->enabled) || (data1 &&data1->ghost_info->enabled))
      {
        return false;
      }
    }

    return (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0 &&
      (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask) != 0;
  } 
}
