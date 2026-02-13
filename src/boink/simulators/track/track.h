#pragma once

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include "boink/simulators/simulator.h"
#include "boink/simulators/track/centerline.h"
#include "boink/simulators/track/ground.h"

#include <memory>
#include <string_view>
#include <vector>

namespace boink
{
  class Track : public Simulator
  {
  public:
    Track(std::string_view filename,
        std::shared_ptr<btDiscreteDynamicsWorld> world);

    void update(btScalar dt) override;
    void updateDebug(Debugger* p_dbg) override;

    const btTransform& getWorldTransform() const {return transform_;}
    void setWorldTransform(const btTransform& position);

    const Centerline& getCenterline() const {return centerline_;}
  private:
    static constexpr std::string_view TRACK_NAME="Sideroad";
    static constexpr std::string_view CENTERLINE_NAME="Centerline";
  private:
    std::vector<Ground> grounds;
    std::shared_ptr<btDiscreteDynamicsWorld> world_;

    Centerline centerline_;
    btTransform transform_=btTransform::getIdentity();
  };
}
