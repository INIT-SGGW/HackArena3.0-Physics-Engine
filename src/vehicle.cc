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

    // TODO
    // move this to CustomRaycastVehicle
    for(const auto&[which,speed] : wheel_speeds_)
    {
      const auto& wheel_info=vehicle_->getWheelInfo((int)which);
      wheel_speeds_[which]=wheel_info.m_deltaRotation/dt;
    }
    
  }

  void Vehicle::updateRender(Renderer* renderer)
  {
    assert(renderer!=nullptr);

    this->updateGui();
  }

  void Vehicle::updateGui()
  {
#ifdef NDEBUG
    VehicleGui* p_vehicle_gui=static_cast<VehicleGui*>(gui_.get());
#else
    VehicleGui* p_vehicle_gui=dynamic_cast<VehicleGui*>(gui_.get());
    assert(p_vehicle_gui!=nullptr);
#endif
    const auto& com=this->getCenterOfMassCS();
    p_vehicle_gui->center_of_mass_cs[0]=com.getX();
    p_vehicle_gui->center_of_mass_cs[1]=com.getY();
    p_vehicle_gui->center_of_mass_cs[2]=com.getZ();

    btTransform chassis_transform=this->getChassisWorldTransform();
    const auto& chassis_pos=chassis_transform.getOrigin();
    p_vehicle_gui->chassis_position[0]=chassis_pos.getX();
    p_vehicle_gui->chassis_position[1]=chassis_pos.getY();
    p_vehicle_gui->chassis_position[2]=chassis_pos.getZ();

    p_vehicle_gui->curr_lap_coverage=this->getCurrentLapDistanceCovered();
    p_vehicle_gui->laps_completed=this->getLapsCompleted();
    p_vehicle_gui->mass=this->getMass();
    p_vehicle_gui->speed=this->getSpeed();

    CustomRaycastVehicle::btVehicleTuning tuning;
    tuning.m_frictionSlip = p_vehicle_gui->friction_slip;
    tuning.m_maxSuspensionForce = p_vehicle_gui->max_suspension_force;
    tuning.m_maxSuspensionTravelCm = p_vehicle_gui->max_suspension_travel_cm;
    tuning.m_suspensionCompression = p_vehicle_gui->suspension_compression;
    tuning.m_suspensionDamping = p_vehicle_gui->suspension_damping;
    tuning.m_suspensionStiffness = p_vehicle_gui->suspension_stiffness;
    this->setTuning(tuning);
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
    return wheel_speeds_.at(wheel_pos);
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

#ifdef NDEBUG
    VehicleGui* p_vehicle_gui=static_cast<VehicleGui*>(gui_.get());
#else
    VehicleGui* p_vehicle_gui=dynamic_cast<VehicleGui*>(gui_.get());
    assert(p_vehicle_gui!=nullptr);
#endif
    p_vehicle_gui->friction_slip=tuning.m_frictionSlip;
    p_vehicle_gui->max_suspension_force=tuning.m_maxSuspensionForce;
    p_vehicle_gui->max_suspension_travel_cm=tuning.m_maxSuspensionTravelCm;
    p_vehicle_gui->suspension_compression=tuning.m_suspensionCompression;
    p_vehicle_gui->suspension_damping=tuning.m_suspensionDamping;
    p_vehicle_gui->suspension_stiffness=tuning.m_suspensionStiffness;
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
    force*=500.;
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
