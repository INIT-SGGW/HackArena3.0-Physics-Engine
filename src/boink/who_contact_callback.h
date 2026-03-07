#pragma once 

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

#include <vector>

namespace boink
{
  class WhoContactCallback : public btCollisionWorld::ContactResultCallback
  {
  public:
    WhoContactCallback(btCollisionObject* me)
      :me_(me)
    {}

    virtual btScalar addSingleResult(
        btManifoldPoint &cp, 
        const btCollisionObjectWrapper *colObj0Wrap, 
        int partId0, 
        int index0, 
        const btCollisionObjectWrapper *colObj1Wrap, 
        int partId1, 
        int index1) override
    {
      (void)cp;
      (void)partId0;
      (void)index0;
      (void)partId1;
      (void)index1;

      const btCollisionObject* other=nullptr;

      if(me_==colObj0Wrap->getCollisionObject())
        other=colObj1Wrap->getCollisionObject();
      else
        other=colObj0Wrap->getCollisionObject();

      hits_.push_back(other);
      return 0;
    }

    const std::vector<const btCollisionObject*>& getHits() const
    {return hits_;}
  private:
    btCollisionObject* me_=nullptr;
    std::vector<const btCollisionObject*> hits_;
  };
}

