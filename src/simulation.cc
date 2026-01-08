#include "boink/simulation.h"

#include <LinearMath/btIDebugDraw.h>
#include "LinearMath/btDefaultMotionState.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

#include "boink/utils/utility.h"

#include <memory>
#include <algorithm>

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
    for (int i=0;i<rigidbodies_.size();i++)
    {
      btRigidBody* body = rigidbodies_[i].get();
      if (body && body->getMotionState())
      {
        delete body->getMotionState();
      }
      //dynamics_world->removeCollisionObject(body);
      dynamics_world->removeRigidBody(body);
      delete body->getCollisionShape();
    }
  }

  void Simulation::registerDebugDrawer(btIDebugDraw* dbg)
  {
    dynamics_world->setDebugDrawer(dbg);
    dbg->setDebugMode(
        btIDebugDraw::DBG_DrawWireframe |
        btIDebugDraw::DBG_DrawConstraints |
        btIDebugDraw::DBG_DrawContactPoints
    );
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
    btCollisionShape* col_shape=
      new btBoxShape(dimensions/2.); // we pass half dims
                                           
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
      (mass, motion_state, col_shape, local_inertia);

    std::shared_ptr<btRigidBody> body (new btRigidBody(rb_info));

		//add the body to the dynamics world
		dynamics_world->addRigidBody(body.get());

		rigidbodies_.push_back(body);
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

    //collision_shapes.push_back(col_shape);
  }

  void Simulation::addCar(const CarModel& car_model)
  {
    auto rigibody=createCarRigidbody(car_model);
    
    std::shared_ptr<btVehicleRaycaster> raycaster(
        new btDefaultVehicleRaycaster(dynamics_world.get()));

    btRaycastVehicle::btVehicleTuning tuning;
    std::shared_ptr<btRaycastVehicle> vehicle(
        new btRaycastVehicle(tuning, rigibody, raycaster.get()),
        [world=dynamics_world.get()](btRaycastVehicle* ptr)
        {
          world->removeVehicle(ptr);

          auto p_rigidbody=ptr->getRigidBody();
          if(p_rigidbody){
            if(p_rigidbody->getMotionState())
              delete p_rigidbody->getMotionState();

            world->removeRigidBody(p_rigidbody);
            delete p_rigidbody->getCollisionShape();
            delete p_rigidbody;
          }

          delete ptr;
        }
    );

    vehicle->setCoordinateSystem(
        0, // right (X)
        1, // up (Y)
        2  // forward (Z)
    );

    dynamics_world->addVehicle(vehicle.get());
    vehicles_.push_back({vehicle,raycaster});

    btVector3 wheelDirectionCS0(0, -1, 0);
    btVector3 wheelAxleCS(-1, 0, 0);

    btScalar suspensionRestLength = 0.2f;
    btScalar wheelRadius = car_model.getWheelRadius();
    btVector3 wheel_center_ajust=
      wheelDirectionCS0*(suspensionRestLength+wheelRadius);

    bool isFrontWheel = true;

    // Front-left
    vehicle->addWheel(
        car_model.getFrontLeftWheel()-wheel_center_ajust,
        wheelDirectionCS0,
        wheelAxleCS,
        suspensionRestLength,
        wheelRadius,
        tuning,
        isFrontWheel
    );

    // Front-right
    vehicle->addWheel(
        car_model.getFrontRightWheel()-wheel_center_ajust,
        wheelDirectionCS0,
        wheelAxleCS,
        suspensionRestLength,
        wheelRadius,
        tuning,
        isFrontWheel
    );

    isFrontWheel = false;

    // Rear-left
    vehicle->addWheel(
        car_model.getRearLeftWheel()-wheel_center_ajust,
        wheelDirectionCS0,
        wheelAxleCS,
        suspensionRestLength,
        wheelRadius,
        tuning,
        isFrontWheel
    );

    // Rear-right
    vehicle->addWheel(
        car_model.getRearRightWheel()-wheel_center_ajust,
        wheelDirectionCS0,
        wheelAxleCS,
        suspensionRestLength,
        wheelRadius,
        tuning,
        isFrontWheel
    );

  }

  [[nodiscard]]
  btRigidBody* Simulation::createCarRigidbody(const CarModel& car_model)
  {
    std::vector<piksel::Mesh::Vertex> mesh_vertices;
    glm::mat4 mesh_transform;
    for(const auto& mesh : car_model.getModel().getMeshes())
    {
      if(mesh.getName()==CarModel::BODY_NAME)
      {
        mesh_vertices=mesh.getVertices();
        mesh_transform=mesh.getTransform();
      }
    }
    std::vector<btVector3> vertices; vertices.reserve(mesh_vertices.size());
    std::transform(
        mesh_vertices.cbegin(),mesh_vertices.cend(),
        std::back_inserter(vertices),
        [](const piksel::Mesh::Vertex& vertex)
        {
          return glm2bt(vertex.pos);
        }
    );
    auto [scale,transform]=glm2bt(mesh_transform);

    // TODO
    // Resolve memory leaks.

    // TODO
    // A single collison shape can be shared across multpile objects
    btConvexHullShape* hull=new btConvexHullShape();

    for (const btVector3& v : vertices)
    {
        hull->addPoint(v, false);
    }

    hull->recalcLocalAabb();
    hull->optimizeConvexHull();
    hull->initializePolyhedralFeatures();

    hull->setLocalScaling(scale);

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool is_dynamic = (car_model.getMass() != 0.f);

    //using motionstate is recommended, 
    //it provides interpolation capabilities, 
    //and only synchronizes 'active' objects
    btVector3 local_inertia(0, 0, 0);
    if (is_dynamic)
      hull->calculateLocalInertia(car_model.getMass(), local_inertia);

    //using motionstate is optional, 
    //it provides interpolation capabilities, 
    //and only synchronizes 'active' objects
    btDefaultMotionState* motion_state = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rb_info
      (car_model.getMass(), motion_state, hull, local_inertia);

    btRigidBody* body =new btRigidBody(rb_info);

    //add the body to the dynamics world
    dynamics_world->addRigidBody(body);

    return body;
  }
}   
    
    
