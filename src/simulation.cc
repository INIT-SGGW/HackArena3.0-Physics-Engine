#include "boink/simulation.h"

#include <LinearMath/btIDebugDraw.h>
#include "LinearMath/btDefaultMotionState.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

#include "boink/simulation/track.h"

#include <memory>
#include <piksel/model.hh>

namespace boink
{
  Simulation::Simulation(
      std::string_view track_filepath)
    :collision_configuration_(new btDefaultCollisionConfiguration()),
    dispatcher_(new btCollisionDispatcher(collision_configuration_.get())),
    overlapping_pair_cache_(new btDbvtBroadphase()),
    solver_(new btSequentialImpulseConstraintSolver()),
    dynamics_world_(new btDiscreteDynamicsWorld(
          dispatcher_.get(),overlapping_pair_cache_.get(),
          solver_.get(),collision_configuration_.get())),
    track_(track_filepath,dynamics_world_)
  {
    dynamics_world_->setGravity(btVector3(0, -GRAVITATIONAL_ACCELERATION, 0));

  } 

  void Simulation::registerDebugDrawer(btIDebugDraw* dbg)
  {
    dynamics_world_->setDebugDrawer(dbg);
    dbg->setDebugMode(
        btIDebugDraw::DBG_DrawWireframe |
        btIDebugDraw::DBG_DrawConstraints |
        btIDebugDraw::DBG_DrawContactPoints |
        btIDebugDraw::DBG_DrawAabb
    );
  }

  void Simulation::step(double dt )
  {
    dynamics_world_->stepSimulation(dt, 5);
    dynamics_world_->debugDrawWorld();
  }

  Simulation::ObjectID Simulation::addCar(const Vehicle::CreationInfo& info)
  {
    vehicles_.emplace_back(info,dynamics_world_);
    return vehicles_.size()-1;
  }

  void Simulation::removeCar(ObjectID id)
  {
    assert(id<vehicles_.size());
    vehicles_.erase(vehicles_.cbegin()+id);
  }

  Vehicle& Simulation::getCar(Simulation::ObjectID id)
  {
    assert(id<vehicles_.size());
    return vehicles_[id];
  }

  Track& Simulation::getTrack()
  {
    return track_;
  }

  void Simulation::addSphere(btScalar radius, const btVector3& origin)
  {
    std::shared_ptr<btCollisionShape> col_shape(new btSphereShape(radius));

    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(origin);

    btScalar mass(1.f);

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool is_dynamic = (mass != 0.f);

    //using motionstate is recommended, 
    //it provides interpolation capabilities, 
    //and only synchronizes 'active' objects
    btVector3 local_inertia(0, 0, 0);
    if (is_dynamic)
      col_shape->calculateLocalInertia(mass, local_inertia);

    //using motionstate is optional, 
    //it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* motion_state = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rb_info
      (mass, motion_state, col_shape.get(), local_inertia);
    btRigidBody* body = new btRigidBody(rb_info);

    //add the body to the dynamics world
    dynamics_world_->addRigidBody(body);

    //collision_shapes.push_back(col_shape);
  }

}   
    
    
