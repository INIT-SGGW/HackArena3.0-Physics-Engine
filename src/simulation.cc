#include "boink/simulation.h"

#include <LinearMath/btIDebugDraw.h>
#include <memory>
#include "LinearMath/btDefaultMotionState.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"

namespace boink
{
  Simulation::Simulation()
    :collision_configuration(new btDefaultCollisionConfiguration()),
    dispatcher(new btCollisionDispatcher(collision_configuration.get())),
    overlapping_pair_cache(new btDbvtBroadphase()),
    solver(new btSequentialImpulseConstraintSolver()),
    dynamics_world(new btDiscreteDynamicsWorld(
          dispatcher.get(),overlapping_pair_cache.get(),
          solver.get(),collision_configuration.get()))
  {
    dynamics_world->setGravity(btVector3(0, -GRAVITATIONAL_ACCELERATION, 0));
  } 

  Simulation::~Simulation() noexcept
  {
    //remove the rigidbodies from the dynamics world and delete them
    for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--)
    {
      btCollisionObject* obj = dynamics_world->getCollisionObjectArray()[i];
      btRigidBody* body = btRigidBody::upcast(obj);
      if (body && body->getMotionState())
      {
        delete body->getMotionState();
      }
      dynamics_world->removeCollisionObject(obj);
      delete obj;
    }
  }

  void Simulation::registerDebugDrawer(btIDebugDraw* dbg)
  {
    dynamics_world->setDebugDrawer(dbg);
  }

  void Simulation::step(double dt)
  {
    dynamics_world->stepSimulation(dt, 5);
    dynamics_world->debugDrawWorld();
  }


  void Simulation::addGround(
      const btVector3& dimensions,
      const btVector3& origin)
  {
    std::shared_ptr<btCollisionShape> col_shape(
      new btBoxShape(dimensions/2.)); // we pass half dims
                                           
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(origin);

		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool is_dynamic = (mass != 0.f);

		btVector3 local_inertia(0, 0, 0);
		if (is_dynamic)
			col_shape->calculateLocalInertia(mass, local_inertia);

		//using motionstate is optional, 
    //it provides interpolation capabilities, 
    //and only synchronizes 'active' objects
		btDefaultMotionState* motion_state = new btDefaultMotionState(transform);
		btRigidBody::btRigidBodyConstructionInfo rb_info
      (mass, motion_state, col_shape.get(), local_inertia);
		btRigidBody* body = new btRigidBody(rb_info);

		//add the body to the dynamics world
		dynamics_world->addRigidBody(body);

		collision_shapes.push_back(col_shape);
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
    dynamics_world->addRigidBody(body);

    collision_shapes.push_back(col_shape);
  }

  Rigidbody Simulation::createCarRigidbody(
      const std::vector<btVector3>& vertices, 
      const btTransform& trans,
      btScalar mass)
  {

    std::shared_ptr<btConvexHullShape> hull(new btConvexHullShape());

    for (const btVector3& v : vertices)
    {
        hull->addPoint(v, false);
    }

    hull->recalcLocalAabb();
    hull->optimizeConvexHull();
    hull->initializePolyhedralFeatures();

    btTransform transform=trans;

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool is_dynamic = (mass != 0.f);

    //using motionstate is recommended, 
    //it provides interpolation capabilities, 
    //and only synchronizes 'active' objects
    btVector3 local_inertia(0, 0, 0);
    if (is_dynamic)
      hull->calculateLocalInertia(mass, local_inertia);

    //using motionstate is optional, 
    //it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* motion_state = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rb_info
      (mass, motion_state, hull.get(), local_inertia);
    std::shared_ptr<btRigidBody> body (new btRigidBody(rb_info));

    //add the body to the dynamics world
    dynamics_world->addRigidBody(body.get());

    collision_shapes.push_back(hull);

    return {mass,local_inertia,hull,body};
  }
}   
    
    
    
    
    
