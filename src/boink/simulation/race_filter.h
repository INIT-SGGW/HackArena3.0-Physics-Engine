#pragma once

#include <BulletCollision/BroadphaseCollision/btOverlappingPairCache.h>
#include <BulletCollision/BroadphaseCollision/btOverlappingPairCallback.h>

namespace boink
{
  class RaceFilter : public btOverlapFilterCallback
  {
  public:
    virtual ~RaceFilter()=default;

    virtual bool needBroadphaseCollision(
        btBroadphaseProxy* proxy0, 
        btBroadphaseProxy* proxy1) const override;
  };
}
