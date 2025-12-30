#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"

#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"

#include "LinearMath/btTransform.h"
#include "LinearMath/btDefaultMotionState.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "piksel/model.hh"
#include "piksel/window.hh"
#include "piksel/graphics.hh"
#include "piksel/camera.hh"
#include "piksel/color.hh"
#include "piksel/config.hh"
//#include "piksel/cube.hh"
#include "piksel/wireframe.hh"

#include <stdio.h>
#include <memory>

class Simulation
{
public:
  Simulation();
  ~Simulation() noexcept;

  void addGround(const btVector3& dims, const btVector3& pos);
  void addSphere(btScalar radius, const btVector3& pos);
  void addVehicle(const btVector3& dims, const btVector3& pos);
  void Step(double dt);
public:
  static constexpr double GRAVITATIONAL_ACCELERATION=10.;
private:
  std::unique_ptr<btDefaultCollisionConfiguration> collision_configuration;
  std::unique_ptr<btCollisionDispatcher> dispatcher;
  std::unique_ptr<btDbvtBroadphase> overlapping_pair_cache;
  std::unique_ptr<btSequentialImpulseConstraintSolver> solver;
public:
  std::unique_ptr<btDiscreteDynamicsWorld> dynamics_world;
  btAlignedObjectArray<std::shared_ptr<btCollisionShape>> collision_shapes;
};

using namespace piksel;
void handleCameraMovement(const Window& wnd, Camera& cam,float cam_speed,float dt);

int main(int argc, char** argv)
{
  constexpr float cam_speed=50.f; 
  Window wnd("Model load test", 1280,720);
  Camera cam({0.f,-56.f,100.f},{0.f,-56.f,0.f});
  Graphics gfx(
      wnd,cam,
      PIKSEL_SHADERS_PATH"/single_color.vert",
      PIKSEL_SHADERS_PATH"/single_color.frag");
  gfx.setBackground(Color::Blue);

  auto car = std::make_shared<Model>("Bolid_F1.glb",1.f);
  car->color=Color::Green;
  float scale_factor=12.25;
  car->scale=glm::scale(car->scale,glm::vec3(1.f)*scale_factor);
  gfx.addObject(car);

  btVector3 ground_dims(100.,100.,100.);
  btVector3 size=ground_dims;
  auto ground=std::make_shared<Wireframe>(5.0f);
  ground->color=Color::Green;
  ground->scale=glm::scale(glm::mat4(1.f),{size.x(),size.y(),size.z()});
  gfx.addObject(ground);

  btScalar radius=1.f;
  btVector3 sphere_dims(radius,radius,radius);
  auto sphere=std::make_shared<Wireframe>(5.0f);
  sphere->color=Color::Red;
  sphere->scale=glm::scale(glm::mat4(1.f),glm::vec3(1.f)*(float)radius);
  gfx.addObject(sphere);
  
  Simulation simulation;
  simulation.addGround(ground_dims,{0.,-56.,0.});
  simulation.addSphere(radius,{50.,50.,0.});

  float last=glfwGetTime();
  while(wnd)
  {
    float right_now=glfwGetTime();
    float dt=right_now-last;
    last=right_now;

    simulation.Step(dt);

    if(wnd.getKey(GLFW_KEY_ESCAPE)==Window::KeyState::Press)
      wnd.close();

    handleCameraMovement(wnd,cam,cam_speed,dt);

    //print positions of all objects
    for (int j = simulation.dynamics_world->getNumCollisionObjects() - 1; j >= 0; j--)
    {
      btCollisionObject* obj = simulation.dynamics_world->getCollisionObjectArray()[j];
      btRigidBody* body = btRigidBody::upcast(obj);
      btTransform trans;
      if (body && body->getMotionState())
      {
        body->getMotionState()->getWorldTransform(trans);
      }
      else
      {
        trans = obj->getWorldTransform();
      }
      btCollisionShape* col_shape=obj->getCollisionShape();

      btVector3 pos=trans.getOrigin();
      btQuaternion quad=trans.getRotation();
      btMatrix3x3 m3(quad);

      glm::mat4 rot(1.0f); // identity
      for (int i = 0; i < 3; ++i)
      {
        for (int k = 0; k < 3; ++k)
        {
          rot[k][i] = m3[i][k]; // row-major
        }
      }
      if(j==0)
      {
        printf("world pos object %d = %f,%f,%f\n", 
            j, float(size.getX()), 
            float(size.getY()), float(size.getZ()));
        ground->translate=glm::translate(glm::mat4(1.f),{pos.x(),pos.y(),pos.z()});
        ground->rotate=rot;
      }
      if(j==1)
      {
        printf("world pos object %d = %f,%f,%f\n", 
            j, float(size.getX()), 
            float(size.getY()), float(size.getZ()));
        sphere->translate=glm::translate(glm::mat4(1.f),{pos.x(),pos.y(),pos.z()});
        sphere->rotate=rot;
      }

      printf("world pos object %d = %f,%f,%f\n", 
          j, float(trans.getOrigin().getX()), 
          float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
    }

    gfx.render();
    wnd.update();
  }
}

void handleCameraMovement(const Window& wnd, Camera& cam, float cam_speed,float dt)
{
  static Window::MousePos prev_mouse_pos=wnd.getMousePos();

  if(wnd.getKey(GLFW_KEY_W)==Window::KeyState::Press){
    cam.moveLongitudinal(cam_speed*dt);
  }
  if(wnd.getKey(GLFW_KEY_S)==Window::KeyState::Press){
    cam.moveLongitudinal(-cam_speed*dt);
  }

  if(wnd.getKey(GLFW_KEY_A)==Window::KeyState::Press){
    cam.moveLateral(-cam_speed*dt);
  }
  if(wnd.getKey(GLFW_KEY_D)==Window::KeyState::Press){
    cam.moveLateral(cam_speed*dt);
  }

  Window::MousePos mouse_pos=wnd.getMousePos();
  cam.rotateYaw((prev_mouse_pos.x-mouse_pos.x)*dt/3.f);
  prev_mouse_pos.x=mouse_pos.x;
  cam.rotatePitch((prev_mouse_pos.y-mouse_pos.y)*dt/3.f);
  prev_mouse_pos.y=mouse_pos.y;
}

Simulation::Simulation()
  : collision_configuration(new btDefaultCollisionConfiguration()),
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

void Simulation::addGround(const btVector3& dims, const btVector3& origin)
{
    std::shared_ptr<btCollisionShape> col_shape(
      new btBoxShape(dims/2.)); // we pass half dims
                                           
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
    //it provides interpolation capabilities, and only synchronizes 'active' objects
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

void Simulation::addVehicle(const btVector3& dims, const btVector3& pos)
{
  btCollisionShape* chassisShape =
    new btBoxShape(btVector3(1.0f, 0.5f, 2.0f));

  btTransform chassisTransform;
  chassisTransform.setIdentity();
  chassisTransform.setOrigin(btVector3(0, 1, 0));

  btScalar mass = 800.0f;
  btVector3 inertia(0, 0, 0);
  chassisShape->calculateLocalInertia(mass, inertia);

  btDefaultMotionState* motionState =
      new btDefaultMotionState(chassisTransform);

  btRigidBody::btRigidBodyConstructionInfo rbInfo(
      mass, motionState, chassisShape, inertia);

  btRigidBody* chassisBody = new btRigidBody(rbInfo);
  dynamics_world->addRigidBody(chassisBody);


  btVehicleRaycaster* raycaster =
      new btDefaultVehicleRaycaster(dynamics_world.get());

  btRaycastVehicle::btVehicleTuning tuning;
  btRaycastVehicle* vehicle =
      new btRaycastVehicle(tuning, chassisBody, raycaster);

  vehicle->setCoordinateSystem(
      0, // right (X)
      1, // up (Y)
      2  // forward (Z)
  );

  dynamics_world->addVehicle(vehicle);
  

  btVector3 wheelDirectionCS0(0, -1, 0);
  btVector3 wheelAxleCS(-1, 0, 0);

  btScalar suspensionRestLength = 0.6f;
  btScalar wheelRadius = 0.5f;

  bool isFrontWheel = true;

  // Front-left
  vehicle->addWheel(
      btVector3(-1.0f, 0.2f,  2.0f),
      wheelDirectionCS0,
      wheelAxleCS,
      suspensionRestLength,
      wheelRadius,
      tuning,
      isFrontWheel
  );

  // Front-right
  vehicle->addWheel(
      btVector3( 1.0f, 0.2f,  2.0f),
      wheelDirectionCS0,
      wheelAxleCS,
      suspensionRestLength,
      wheelRadius,
      tuning,
      isFrontWheel
  );

  isFrontWheel = false;

  // Rear wheels
  vehicle->addWheel(
      btVector3(-1.0f, 0.2f, -2.0f),
      wheelDirectionCS0,
      wheelAxleCS,
      suspensionRestLength,
      wheelRadius,
      tuning,
      isFrontWheel
  );

  vehicle->addWheel(
      btVector3( 1.0f, 0.2f, -2.0f),
      wheelDirectionCS0,
      wheelAxleCS,
      suspensionRestLength,
      wheelRadius,
      tuning,
      isFrontWheel
  );
}

void Simulation::Step(double dt)
{
  dynamics_world->stepSimulation(dt, 5);

}
