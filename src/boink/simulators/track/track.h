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
#include "boink/gui/track_gui.h"

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
    void updateRender(Renderer* p_renderer) override;
    std::shared_ptr<piksel::GuiObject> getGui() override {return gui_;}

    const btTransform& getWorldTransform() const {return transform_;}
    void setWorldTransform(const btTransform& position);

    const Centerline& getCenterline() const {return centerline_;}
    std::string_view getFilename() const { return filename_;}
  private:
    void updateGui();
  private:
    static constexpr std::string_view TRACK_NAME="Sideroad";
    static constexpr std::string_view CENTERLINE_NAME="Centerline";
  private:
    std::vector<Ground> grounds;
    std::shared_ptr<btDiscreteDynamicsWorld> world_;

    Centerline centerline_;
    btTransform transform_=btTransform::getIdentity();
    std::string_view filename_;
    std::shared_ptr<TrackGui> gui_;
  };
}
