#include "boink/simulation/vehicle.h"
#include "boink/simulation/wheel_position.h"

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>

#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

#include <LinearMath/btDefaultMotionState.h>

#include <memory>
#include <cassert>

namespace boink
{
  Vehicle::Vehicle(
      const CreationInfo& create_info,
      std::shared_ptr<btDynamicsWorld> world)
    :
      mesh_(create_info.mesh),
      world_(world),
      motion_state_(new btDefaultMotionState(mesh_->getChassis().transform)),
      raycaster_(new btDefaultVehicleRaycaster(world_.get())),
      center_of_mass_(create_info.center_of_mass),
      max_steer_angle_(create_info.max_steer_angle)
  {
    collision_shape_=createCollisonShape(
        mesh_->getChassis().vertices,
        center_of_mass_);
    rigidbody_=createRigidbody(create_info.mass);

    // I dont know why but everybody does this.
    rigidbody_->setActivationState(DISABLE_DEACTIVATION);

    vehicle_=std::unique_ptr<btRaycastVehicle>(
        new btRaycastVehicle(tuning_, rigidbody_.get(), raycaster_.get())
    );

    vehicle_->setCoordinateSystem(
        0, // right (X)
        1, // up (Y)
        2  // forward (Z)
    );

    world_->addVehicle(vehicle_.get());

    btVector3 wheel_direction_cs0(0, -1, 0);
    btVector3 wheel_axle_cs(-1, 0, 0);

    btScalar suspension_rest_length=create_info.suspension_rest_length;
    btScalar wheel_radius=create_info.wheel_radius;

    bool is_front_wheel=false;

    // ORDER OF CREATION OF THE WHEELS MUST MATCH WITH WHEELPOSITION ENUM

    // Rear-left
    vehicle_->addWheel(
        mesh_->getLocalWheelTransform(WheelPosition::RearLeft).getOrigin()
        +center_of_mass_,
        wheel_direction_cs0,
        wheel_axle_cs,
        suspension_rest_length,
        wheel_radius,
        tuning_,
        is_front_wheel
    );

    // Rear-right
    vehicle_->addWheel(
        mesh_->getLocalWheelTransform(WheelPosition::RearRight).getOrigin()
        +center_of_mass_,
        wheel_direction_cs0,
        wheel_axle_cs,
        suspension_rest_length,
        wheel_radius,
        tuning_,
        is_front_wheel
    );

    is_front_wheel=true;

    // Front-left
    vehicle_->addWheel(
        mesh_->getLocalWheelTransform(WheelPosition::FrontLeft).getOrigin()
        +center_of_mass_,
        wheel_direction_cs0,
        wheel_axle_cs,
        suspension_rest_length,
        wheel_radius,
        tuning_,
        is_front_wheel
    );

    // Front-right
    vehicle_->addWheel(
        mesh_->getLocalWheelTransform(WheelPosition::FrontRight).getOrigin()
        +center_of_mass_,
        wheel_direction_cs0,
        wheel_axle_cs,
        suspension_rest_length,
        wheel_radius,
        tuning_,
        is_front_wheel
    );
  }

  Vehicle::~Vehicle() noexcept
  {
    if(vehicle_)
    {
      world_->removeVehicle(vehicle_.get());
    }
    
    if(rigidbody_)
      world_->removeRigidBody(rigidbody_.get());

    if(collision_shape_)
    {
      for(int i=0;i<collision_shape_->getNumChildShapes();i++)
      {
        delete collision_shape_->getChildShape(i);
      }
    }
  }

  void Vehicle::setPosition(const btVector3& position)
  {
    rigidbody_->getWorldTransform().setOrigin(position);
  }

  btTransform Vehicle::getWorldTransform() const
  {
    btTransform transform;
    motion_state_->getWorldTransform(transform);
    return transform;
  }

  btTransform Vehicle::getChassisWorldTransform() const
  {
    // Because we moved out center of mass via
    // compund shape we have to move also the chassis.
    btTransform translate;
    translate.setIdentity();
    translate.setOrigin(center_of_mass_);

    // we must translate before rotation
    return vehicle_->getChassisWorldTransform()*translate;
  }

  const btTransform& Vehicle::getWheelWorldTransform(WheelPosition wheel_pos) const
  {
    return vehicle_->getWheelTransformWS((int)wheel_pos);
  }

  const btTransform& Vehicle::getCenterOfMassTransform() const
  {
    return rigidbody_->getCenterOfMassTransform();
  }

  btScalar Vehicle::getSpeed() const
  {
    return rigidbody_->getLinearVelocity().length();
  }

  void Vehicle::setSteering(btScalar value, TurnDirection dir)
  {
    btScalar radians=value*max_steer_angle_;
    if(dir==TurnDirection::Right)
      radians*=-1;

    // User should always set value to [0.1]

    vehicle_->setSteeringValue(radians,(int)WheelPosition::FrontLeft);
    vehicle_->setSteeringValue(radians,(int)WheelPosition::FrontRight);
  }

  void Vehicle::setEngineForce(btScalar force)
  {
    vehicle_->applyEngineForce(force,(int)WheelPosition::RearLeft);
    vehicle_->applyEngineForce(force,(int)WheelPosition::RearRight);
  }

  void Vehicle::setBrake(btScalar brake)
  {
    vehicle_->setBrake(brake,(int)WheelPosition::RearLeft);
    vehicle_->setBrake(brake,(int)WheelPosition::RearRight);
    //vehicle_->setBrake(brake,(int)WheelPosition::FrontLeft);
    //vehicle_->setBrake(brake,(int)WheelPosition::FrontRight);
  }

  std::unique_ptr<btCompoundShape> Vehicle::createCollisonShape(
      const std::vector<btVector3>& vertices,
      const btVector3& center_of_mass)
  {
    std::unique_ptr<btCompoundShape> compound(new btCompoundShape());
    btConvexHullShape* hull=new btConvexHullShape();

    for (const btVector3& v : vertices)
    {
        hull->addPoint(v, false);
    }

    hull->recalcLocalAabb();
    hull->optimizeConvexHull();
    hull->initializePolyhedralFeatures();

    btTransform localTransform;
		localTransform.setIdentity();
		localTransform.setOrigin(center_of_mass);

		//The center of gravity of the compound shape is the origin. 
    //When we add a rigidbody to the compound shape
		//it's center of gravity does not change. 
    //This way we can add the chassis rigidbody one unit above our center of gravity
		//keeping it under our chassis, and not in the middle of it
		compound->addChildShape(localTransform, hull);

    return compound;
  }

  std::unique_ptr<btRigidBody> Vehicle::createRigidbody(
      btScalar mass)
  {
    assert(mass!=0.f);

    btVector3 local_inertia(0, 0, 0);
    collision_shape_->calculateLocalInertia(mass, local_inertia);

    btRigidBody::btRigidBodyConstructionInfo rb_info
      (mass, motion_state_.get(), collision_shape_.get(), local_inertia);

    std::unique_ptr<btRigidBody> body (new btRigidBody(rb_info));

    world_->addRigidBody(body.get());

    return body;
  }

}
