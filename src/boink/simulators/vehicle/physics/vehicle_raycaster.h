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
#pragma once
 
#include <LinearMath/btVector3.h>

class btDynamicsWorld;
class btRigidBody;
namespace boink
{
  class VehicleRaycaster
  {
  public:
    struct VehicleRaycasterResult
    {
      VehicleRaycasterResult() 
        : m_distFraction(btScalar(-1.))
      {}

      btVector3 m_hitPointInWorld;
      btVector3 m_hitNormalInWorld;
      btScalar m_distFraction;
    };
  public:
    VehicleRaycaster(btDynamicsWorld* world);

    btRigidBody* castRay(
        const btVector3& from, 
        const btVector3& to, 
        VehicleRaycasterResult& result);
  private:
    btDynamicsWorld* m_dynamicsWorld=nullptr;
  };
}
