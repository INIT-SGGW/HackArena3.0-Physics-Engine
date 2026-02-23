#include "boink/simulators/vehicle/vehicle.h"
#include "boink/simulators/vehicle/custom_raycast_vehicle.h"
#include "boink/simulators/vehicle/wheel_position.h"
#include "boink/gui/vehicle_gui.h"

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>

#include <LinearMath/btDefaultMotionState.h>

#include <memory>
#include <cassert>

namespace boink
{
  Vehicle::Vehicle(
      const CreationInfo& create_info,
      std::shared_ptr<const Track> track,
      std::shared_ptr<btDynamicsWorld> world)
    :
      mesh_(create_info.mesh),
      world_(world),
      motion_state_(new btDefaultMotionState(mesh_->getChassis().transform)),
      raycaster_(new btDefaultVehicleRaycaster(world_.get())),
      track_(track),
      center_of_mass_(create_info.center_of_mass),
      max_steer_angle_(create_info.max_steer_angle),
      tuning_(create_info.tuning),
      gui_(std::make_shared<VehicleGui>())
  {
    this->correctCOM();
    collision_shape_=createCollisonShape(
        mesh_->getChassis().vertices,
        center_of_mass_);
    rigidbody_=createRigidbody(create_info.mass);

    // I dont know why but everybody does this.
    rigidbody_->setActivationState(DISABLE_DEACTIVATION);

    // Because cars might move fast we wanna avoid
    // cliping them or just going over a wall
    rigidbody_->setCcdMotionThreshold(1.0);
    rigidbody_->setCcdSweptSphereRadius(0.5);

    vehicle_=std::unique_ptr<CustomRaycastVehicle>(
        new CustomRaycastVehicle(tuning_, rigidbody_.get(), raycaster_.get())
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
        -center_of_mass_,
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
        -center_of_mass_,
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
        -center_of_mass_,
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
        -center_of_mass_,
        wheel_direction_cs0,
        wheel_axle_cs,
        suspension_rest_length,
        wheel_radius,
        tuning_,
        is_front_wheel
    );

    this->setTuning(tuning_);
    
    gui_->tunning=&tuning_;
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

  void Vehicle::update(btScalar dt)
  {
    (void)dt;
    btScalar track_length=track_->getCenterline().getLength();

    int curr_laps_completed=this->getLapsCompleted();

    const btVector3 vehicle_pos=this->getWorldTransform().getOrigin();
    btScalar prev_coverage=this->getCurrentLapDistanceCovered();
    btScalar curr_coverage=track_->getCenterline().getCoverage(vehicle_pos);

    btScalar v=curr_coverage-prev_coverage;
    if(btFabs(v)>track_length/2.)
    {
      // Means that finish line was crossed
      if(v>0)
        curr_laps_completed--;
      else
        curr_laps_completed++;
    }

    laps_completed_=curr_laps_completed;
    curr_lap_dist_point_=curr_coverage;
  }

  void Vehicle::updateRender(Renderer* renderer)
  {
    if(renderer==nullptr)
      return;

    this->updateGui();
  }

  void Vehicle::updateGui()
  {
    const auto& com=this->getCenterOfMassCS();
    gui_->center_of_mass_cs[0]=com.getX();
    gui_->center_of_mass_cs[1]=com.getY();
    gui_->center_of_mass_cs[2]=com.getZ();

    btTransform chassis_transform=this->getChassisWorldTransform();
    const auto& chassis_pos=chassis_transform.getOrigin();
    gui_->chassis_position[0]=chassis_pos.getX();
    gui_->chassis_position[1]=chassis_pos.getY();
    gui_->chassis_position[2]=chassis_pos.getZ();

    gui_->curr_lap_coverage=this->getCurrentLapDistanceCovered();
    gui_->laps_completed=this->getLapsCompleted();
    gui_->mass=this->getMass();
    gui_->speed=this->getSpeed();

    this->setTuning(tuning_);
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
    translate.setOrigin(-center_of_mass_);
    btTransform transform;
    // we must translate before rotation
    //return vehicle_->getChassisWorldTransform()*translate;
    motion_state_->getWorldTransform(transform);

    return transform*translate;
  }

  const btTransform& Vehicle::getWheelWorldTransform(WheelPosition wheel_pos) const
  {
    // Wheels
    return vehicle_->getWheelTransformWS((int)wheel_pos);
  }

  btScalar Vehicle::getWheelAngularSpeed(WheelPosition wheel_pos) const
  {
    return vehicle_->getWheelAngularSpeed(wheel_pos);
  }

  const btTransform& Vehicle::getCenterOfMassTransform() const
  {
    return rigidbody_->getCenterOfMassTransform();
  }

  btScalar Vehicle::getSpeed() const
  {
    return rigidbody_->getLinearVelocity().length();
  }

  btScalar Vehicle::getMass() const
  {
    return rigidbody_->getMass();
  }

  btVector3 Vehicle::getCenterOfMassCS() const
  {
    return center_of_mass_;
  }

  void Vehicle::setTuning(const CustomRaycastVehicle::btVehicleTuning& tuning)
  {
    assert(vehicle_->getNumWheels()==4);
    tuning_=tuning;

    for(int i=0;i<vehicle_->getNumWheels();i++)
    {
      btWheelInfo& wheel = vehicle_->getWheelInfo(i);
      wheel.m_suspensionStiffness = tuning_.m_suspensionStiffness;
      wheel.m_wheelsDampingRelaxation = tuning_.m_suspensionDamping;
      wheel.m_wheelsDampingCompression = tuning_.m_suspensionCompression;
      wheel.m_maxSuspensionTravelCm = tuning_.m_maxSuspensionTravelCm;
      wheel.m_maxSuspensionForce=tuning_.m_maxSuspensionForce;
      wheel.m_frictionSlip=tuning_.m_frictionSlip;

      // Some magic number
      wheel.m_rollInfluence=btScalar(0.1);
    }
  }

  const CustomRaycastVehicle::btVehicleTuning& Vehicle::getTuning() const
  {
    assert(vehicle_->getNumWheels()==4);
    return tuning_;
  }

  void Vehicle::setSteering(btScalar value, TurnDirection dir)
  {
    btScalar radians=value*max_steer_angle_;
    if(dir==TurnDirection::Right)
      radians*=-1;

    // User should always set value to [0-1]

    vehicle_->setSteeringValue(radians,(int)WheelPosition::FrontLeft);
    vehicle_->setSteeringValue(radians,(int)WheelPosition::FrontRight);
  }

  void Vehicle::setEngineForce(btScalar force)
  {
    // TODO
    // Make it much smarter
    force*=5000.;
    if(force<0.0f)
      force/=2.f;

    vehicle_->applyEngineForce(force,(int)WheelPosition::RearLeft);
    vehicle_->applyEngineForce(force,(int)WheelPosition::RearRight);
  }

  void Vehicle::setBrake(btScalar brake)
  {
    brake*=40.;
    vehicle_->setBrake(brake,(int)WheelPosition::RearLeft);
    vehicle_->setBrake(brake,(int)WheelPosition::RearRight);
    vehicle_->setBrake(brake,(int)WheelPosition::FrontLeft);
    vehicle_->setBrake(brake,(int)WheelPosition::FrontRight);
  }

  void Vehicle::correctCOM()
  {
    btVector3 front_left_cs=
      mesh_->getLocalWheelTransform(WheelPosition::FrontLeft).getOrigin();
    btVector3 front_right_cs=
      mesh_->getLocalWheelTransform(WheelPosition::FrontRight).getOrigin();
    btVector3 rear_left_cs=
      mesh_->getLocalWheelTransform(WheelPosition::RearLeft).getOrigin();
    btVector3 rear_right_cs=
      mesh_->getLocalWheelTransform(WheelPosition::RearRight).getOrigin();
    btVector3 mid_front=front_left_cs+(front_right_cs-front_left_cs)/2.f;
    btVector3 mid_rear=rear_left_cs+(rear_right_cs-rear_left_cs)/2.f;

    btVector3 mid_point=mid_rear+(mid_front-mid_rear)/2.f;
    mid_point.setY(0.f);

    center_of_mass_+=mid_point;
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
		localTransform.setOrigin(-center_of_mass);

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
