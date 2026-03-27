// clang-format off
#include "boink/simulators/vehicle/vehicle.h"

#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <LinearMath/btDefaultMotionState.h>

#include <LinearMath/btQuaternion.h>
#include <algorithm>
#include <piksel/object.hh>

#include "boink/constants.h"
#include "boink/exception.h"
#include "boink/gui/vehicle_gui.h"
#include "boink/simulators/vehicle/wheel_position.h"
#include "boink/timer.h"
#include "boink/utility.h"
#include "boink/collision_group.h"
#include "boink/assert.h"

#include <memory>

namespace boink
{
Vehicle::Vehicle(const CreationInfo& create_info, std::shared_ptr<const Track> track,
                 std::shared_ptr<btDynamicsWorld> world)
    : mesh_(create_info.mesh),
      world_(world),
      center_of_mass_(
          correctCOM(create_info.center_of_mass,mesh_.get())),
      collision_shape_(
          createCollisonShape(mesh_->getChassis().vertices,center_of_mass_)),
      motion_state_(
          new btDefaultMotionState(mesh_->getChassis().transform)),
      rigidbody_(
          createRigidbody(collision_shape_.get(),motion_state_.get(),create_info.mass)),
      raycaster_(
        std::make_unique<VehicleRaycaster>(world_.get(),rigidbody_.get())),
      vehicle_(new RaycastVehicle(rigidbody_.get(), raycaster_.get())),
      track_(track),
      max_steer_angle_(create_info.max_steer_angle),
      tuning_(create_info.tuning),
      ghost_sim_(world_.get(),vehicle_.get(),&lap_info_),
      pitstop_timer_(kBrakingDuration),
      user_data_(&ghost_info_),
      gui_(std::make_shared<VehicleGui>(this))
  {
    pitstop_timer_.setElapsedToFinish();

    world_->addRigidBody(
        rigidbody_.get(),
        Collision::Group::Vehicle,
        Collision::Group::Vehicle | Collision::Group::Static);

    rigidbody_->setRestitution(0);
    rigidbody_->setUserPointer(&user_data_);
    //
    // I dont know why but everybody does this.
    rigidbody_->setActivationState(DISABLE_DEACTIVATION);

    // Because cars might move fast we wanna avoid
    // cliping or just going over a wall
    rigidbody_->setCcdMotionThreshold(1e-5f);
    rigidbody_->setCcdSweptSphereRadius(0.5f);

    vehicle_->setCoordinateSystem(
        0, // right (X)
        1, // up (Y)
        2  // forward (Z)
    );

  world_->addAction(vehicle_.get());

  btVector3 wheel_direction_cs0(0, -1, 0);
  btVector3 wheel_axle_cs(-1, 0, 0);

  btScalar suspension_rest_length = create_info.suspension_rest_length;
  btScalar wheel_radius = create_info.wheel_radius;

  bool is_front_wheel = false;

  // ORDER OF CREATION OF THE WHEELS MUST MATCH WITH WHEELPOSITION ENUM

  // Rear-left
  vehicle_->addWheel(mesh_->getLocalWheelTransform(WheelPosition::RearLeft).getOrigin() - center_of_mass_,
                     wheel_direction_cs0, wheel_axle_cs, suspension_rest_length, wheel_radius, tuning_, is_front_wheel,
                     create_info.tyre_type);

  // Rear-right
  vehicle_->addWheel(mesh_->getLocalWheelTransform(WheelPosition::RearRight).getOrigin() - center_of_mass_,
                     wheel_direction_cs0, wheel_axle_cs, suspension_rest_length, wheel_radius, tuning_, is_front_wheel,
                     create_info.tyre_type);

  is_front_wheel = true;

  // Front-left
  vehicle_->addWheel(mesh_->getLocalWheelTransform(WheelPosition::FrontLeft).getOrigin() - center_of_mass_,
                     wheel_direction_cs0, wheel_axle_cs, suspension_rest_length, wheel_radius, tuning_, is_front_wheel,
                     create_info.tyre_type);

  // Front-right
  vehicle_->addWheel(mesh_->getLocalWheelTransform(WheelPosition::FrontRight).getOrigin() - center_of_mass_,
                     wheel_direction_cs0, wheel_axle_cs, suspension_rest_length, wheel_radius, tuning_, is_front_wheel,
                     create_info.tyre_type);

  this->setTuning(tuning_);

  bounding_dimensions_=getBoundingDims(collision_shape_);

  const auto& road=track_->getRoad();

  if(!road.isClosed())
      throw Exception(
          Exception::Type::InternalError,
          "Main road should be closed");

  size_t n=road.getSize(Road::Side::Center);
  if(n<2)
      throw Exception(
          Exception::Type::InternalError,
          "Center line has too few points");

  const btVector3& last=road.getPoint(n-1);
  const btVector3& first=road.getPoint(0);

  btVector3 segment=first-last;

  if(segment.length2()<g_Epsilon)
      throw Exception(
          Exception::Type::InternalError,
          "Center line cannot have dupliacted points");

  // g_Epsilon is too small when vehicle tilts a little
  btVector3 before_finish_point=
      last+ 0.99f*segment;

  lap_info_.curr_lap_coverage=
      road.getCoverage(before_finish_point);

  btVector3 up_compensate =
    g_Up * (getChassisToGroundDist() + g_GroundMargin);

  const auto& sample = road.getMetrics(0);
  btQuaternion align_to_surface = shortestArcQuat(g_Up, sample.normal);
  btVector3 local_forward = quatRotate(align_to_surface, boink::g_Forward);

  btQuaternion align_to_tangent = shortestArcQuat(local_forward, sample.tangent);
  btQuaternion final_rot = align_to_tangent * align_to_surface;

  btTransform transform;
  transform.setIdentity();
  transform.setRotation(final_rot);
  transform.setOrigin(before_finish_point+up_compensate);

  this->setChassisWorldTransform(transform);
}

Vehicle::~Vehicle() noexcept
{
  if (vehicle_)
    world_->removeAction(vehicle_.get());

  if (rigidbody_) 
    world_->removeRigidBody(rigidbody_.get());

  if (collision_shape_)
  {
    for (int i = 0; i < collision_shape_->getNumChildShapes(); i++)
      delete collision_shape_->getChildShape(i);
  }
}

void Vehicle::update(btScalar dt)
{
  this->updateLapInfo(dt);

  ghost_sim_.update(dt);

  this->updatePitstop(dt);

  btVector3 chassis=this->getChassisWorldTransform().getOrigin();
  if(chassis.y()<-1000)
    this->setVehicleToPitstop(Pitstop::Zone::Fix);
}

void Vehicle::setVehicleToPitstop(Pitstop::Zone zone)
{
  const auto& fix_road=
    track_->getPitstop().getZone(zone);
  const auto& fix_line_center=fix_road.getLine(boink::Road::Side::Center);

  btVector3 new_pos=fix_line_center.getPoint(fix_line_center.getPointsSize()/2);

  const auto& metrics=fix_road.getClosestMetrics(new_pos);

  btVector3 up_compensate=
    metrics.normal*(getChassisToGroundDist()+boink::g_GroundMargin);
  new_pos+=up_compensate;
  
  btQuaternion align_to_surface = shortestArcQuat(boink::g_Up, metrics.normal);
  btVector3 local_forward = quatRotate(align_to_surface, boink::g_Forward);

  btQuaternion align_to_tangent = shortestArcQuat(local_forward, metrics.tangent);
  btQuaternion final_rot = align_to_tangent * align_to_surface;

  btTransform bt_transform;
  bt_transform.setIdentity();
  bt_transform.setOrigin(new_pos);
  bt_transform.setRotation(final_rot);
  setChassisWorldTransform(bt_transform);
}

void Vehicle::updateLapInfo(btScalar dt)
{
  btScalar track_length = track_->getRoad().getLength();

  int curr_lap= lap_info_.current_lap;

  const btVector3 vehicle_pos = this->getChassisWorldTransform().getOrigin();
  btScalar prev_coverage = lap_info_.curr_lap_coverage;
  btScalar curr_coverage = track_->getRoad().getCoverage(vehicle_pos);

  btScalar v = curr_coverage - prev_coverage;
  if (btFabs(v) > track_length / 2.)
  {
    // Means that finish line was crossed
    if (v > 0)
      curr_lap--;
    else
    {
      // Only here we calculate the time
      btScalar curr_distance=track_length+v;
      BOINK_ASSERT(curr_distance>=0.f);

      if(curr_distance<g_Epsilon)
        curr_distance=g_Epsilon;

      lap_info_.curr_lap_time+=dt*(track_length-prev_coverage)/curr_distance;

      // Vehicle can ride this multiple times so we only save the first one
      if(curr_lap>=LapInfo::kStartingLap&&
          lap_info_.lap_times_history.try_emplace(
          curr_lap,lap_info_.curr_lap_time).second)
        lap_info_.curr_lap_time=0;

      curr_lap++;

      // Update dt we need to short it
      dt=dt*(curr_coverage/curr_distance);
    }
  }
  
  BOINK_ASSERT(dt>=0);
  lap_info_.curr_lap_time+=dt;

  lap_info_.current_lap = curr_lap;
  lap_info_.curr_lap_coverage = curr_coverage;
}

void Vehicle::updatePitstop(btScalar dt)
{
  if(this->isVehicleInPitstop(Pitstop::Zone::Fix)>0)
  {
    if(!ghost_sim_.isInGhostMode())
      ghost_sim_.enterGhostModeForce();

    btVector3 vel=rigidbody_->getLinearVelocity();
    btScalar speed2=vel.length2();

    if(speed2>kMaxFixZoneSpeed*kMaxFixZoneSpeed)
      pitstop_timer_.reset();

    if(!pitstop_timer_.hasFinised() && speed2>kMaxFixZonePenaltySpeed*kMaxFixZonePenaltySpeed)
    {
      btVector3 brake_dir=-vel.normalized();
      rigidbody_->applyCentralImpulse(brake_dir*kPitstopBrakingForce*dt);
    }
  }

  pitstop_timer_.update(dt);
}

void Vehicle::updateRender(Renderer* renderer)
{
  if (renderer == nullptr) return;

  if (!gui_) return;

  if (gui_->mesh_enabled)
  {
    auto chassis_obj =
        std::make_shared<piksel::Object>(mesh_->getChassisPikselMesh(), math::bt2glm(this->getChassisWorldTransform()));
    renderer->addDrawable(chassis_obj);

    for (int i = 0; i < (int)WheelPosition::Count; i++)
    {
      WheelPosition pos = (WheelPosition)i;

      auto wheel_obj =
          std::make_shared<piksel::Object>(mesh_->getWheelPikselMesh(pos), math::bt2glm(this->getWheelWorldTransform(pos)));
      renderer->addDrawable(wheel_obj);
    }
  }

  if (!gui_->collider_enabled)
  {
    rigidbody_->setCollisionFlags(rigidbody_->getCollisionFlags() | btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
  }
  else
  {
    rigidbody_->setCollisionFlags(rigidbody_->getCollisionFlags() & ~btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
  }

  vehicle_->enableDraw(gui_->collider_enabled);

  btVector3 forward_axle = btVector3(
      rigidbody_->getWorldTransform().getBasis()[0][2],
      rigidbody_->getWorldTransform().getBasis()[1][2],
      rigidbody_->getWorldTransform().getBasis()[2][2]);
  btVector3 up_axle = btVector3(
      rigidbody_->getWorldTransform().getBasis()[0][1],
      rigidbody_->getWorldTransform().getBasis()[1][1],
      rigidbody_->getWorldTransform().getBasis()[2][1]);

  btVector3 chassis_center=this->getChassisWorldTransform().getOrigin();
  //renderer->drawLine(
  //    chassis_center,
  //    chassis_center-1*this->getChassisToGroundDist()*g_Up,
  //    {0.5,1.0,0.75f});
  renderer->drawPoint(
      chassis_center,{1,1,1},-forward_axle,-up_axle);
}

std::shared_ptr<piksel::GuiObject> Vehicle::getGui() 
{ 
  return gui_; 
}

void Vehicle::setChassisWorldTransform(const btTransform& transform) 
{ 
  btTransform offset(btQuaternion::getIdentity(),center_of_mass_);
  btTransform new_transform=transform*offset;
  rigidbody_->setWorldTransform(new_transform);
  motion_state_->setWorldTransform(new_transform);

  this->reset();
}

btTransform Vehicle::getChassisWorldTransform() const
{
  // Because we moved out center of mass via
  // compund shape we have to move also the chassis.
  btTransform translate;
  translate.setIdentity();
  translate.setOrigin(-center_of_mass_);
  btTransform transform=rigidbody_->getWorldTransform();

  // we must translate after rotation
  return transform * translate;
}

btScalar Vehicle::getChassisToGroundDist() const
{
  BOINK_ASSERT((btVector3(0.f,1.f,0.f)-g_Up).length2()<g_Epsilon);

  // TODO i dont know but this function is not ideal
  const auto& wheel_info=vehicle_->getWheelInfo((int)WheelPosition::RearLeft);
  return 
    -wheel_info.m_suspensionInfo.m_chassisConnectionPointCS.y()+
    wheel_info.m_suspensionInfo.m_restLength+
    wheel_info.m_wheelsRadius-center_of_mass_.y();
}

const btTransform& Vehicle::getWheelWorldTransform(WheelPosition wheel_pos) const
{
  // Wheels
  return vehicle_->getWheelTransformWS((int)wheel_pos);
}

btScalar Vehicle::getEngineRPM() const { return vehicle_->getEngineRPM();}

int Vehicle::getCurrentGear() const { return vehicle_->getCurrentGear(); }

btScalar Vehicle::getWheelAngularSpeed(WheelPosition wheel_pos) const
{
  return vehicle_->getWheelInfo((int)wheel_pos).m_angSpeed;
}

const btTransform& Vehicle::getCenterOfMassTransform() const { return rigidbody_->getCenterOfMassTransform(); }

bool Vehicle::areAllWheelsOnGround() const
{
  for(int i=0;i<(int)WheelPosition::Count;i++)
  {
    if(!vehicle_->getWheelInfo(i).m_raycastInfo.m_isInContact)
      return false;
  }

  return true;
}

btVector3 Vehicle::getVehicleDirection() const
{
  btQuaternion quat=getChassisWorldTransform().getRotation();
  
  return quatRotate(quat,g_Forward).normalized();
}

btScalar Vehicle::getSpeed() const { return rigidbody_->getLinearVelocity().length(); }

btScalar Vehicle::getMass() const { return rigidbody_->getMass(); }

btVector3 Vehicle::getCenterOfMassCS() const { return center_of_mass_; }

btScalar Vehicle::getTyreHealth(WheelPosition pos) const
{
  return vehicle_->getWheelInfo((int)pos).m_tyreInfo.m_health;
}

WheelInfo::TyreType Vehicle::getTyreType(WheelPosition pos) const
{
  return vehicle_->getWheelInfo((int)pos).m_tyreInfo.m_type;
}

btScalar Vehicle::getTyreTempCelsius(WheelPosition pos) const
{
  return vehicle_->getWheelInfo((int)pos).m_tyreInfo.m_tempCelsius;
}

btScalar Vehicle::getTyreSlipLen(WheelPosition pos) const {
    return vehicle_->getWheelInfo((int)pos).m_slip_vec_length;
}

void Vehicle::setTuning(const RaycastVehicle::VehicleTuning& tuning)
{
  BOINK_ASSERT(getNumWheels() == 4);
  tuning_ = tuning;

  for (int i = 0; i < getNumWheels(); i++)
  {
    WheelInfo& wheel = vehicle_->getWheelInfo(i);
    wheel.m_suspensionInfo.m_stiffness = tuning_.m_suspensionStiffness;
    wheel.m_suspensionInfo.m_wheelsDampingRelaxation = tuning_.m_suspensionDamping;
    wheel.m_suspensionInfo.m_wheelsDampingCompression = tuning_.m_suspensionCompression;
    wheel.m_suspensionInfo.m_maxTravelCm = tuning_.m_maxSuspensionTravelCm;
    wheel.m_suspensionInfo.m_maxForce = tuning_.m_maxSuspensionForce;
    wheel.m_frictionSlip = tuning_.m_frictionSlip;

    // Some magic number
    wheel.m_rollInfluence = btScalar(0.1);
  }
}

const RaycastVehicle::VehicleTuning& Vehicle::getTuning() const
{
  BOINK_ASSERT(getNumWheels() == 4);
  return tuning_;
}

std::pair<btScalar,Vehicle::TurnDirection> Vehicle::getSteering(
    WheelPosition pos) const
{
  btScalar rad=vehicle_->getSteeringValue((int)pos);

  TurnDirection dir=TurnDirection::Left;
  if(rad<0)
  {
    dir=TurnDirection::Right;
    rad*=-1;
  }

  return {rad,dir};
}

void Vehicle::setSteering(btScalar value, TurnDirection dir)
{
  btScalar radians = value * max_steer_angle_;
  if (dir == TurnDirection::Right) radians *= -1;

  // User should always set value to [0-1]

  // only for debugging purpose, delete in future
  vehicle_->m_steeringValue = radians;

  vehicle_->setSteeringValue(radians, (int)WheelPosition::FrontLeft);
  vehicle_->setSteeringValue(radians, (int)WheelPosition::FrontRight);
}

void Vehicle::setEngineForce(btScalar force) { vehicle_->m_throttle = force; }

void Vehicle::setBrake(btScalar brake)
{
  vehicle_->m_brake = brake;
}

void Vehicle::setBrakeBias(btScalar bias) {
    vehicle_->m_brakeBias = bias;
}

void Vehicle::setDiffSetting(btScalar diffsetting) {
    vehicle_->m_diffSetting = diffsetting;
}

void Vehicle::setTyreType(WheelInfo::TyreType tyre_type) {
    vehicle_->setTyreType(tyre_type);
}

bool Vehicle::setGearDown() { return vehicle_->setGearDown(); }

bool Vehicle::setGearUp() { return vehicle_->setGearUp(); }

void Vehicle::enableGhostSim(const GhostModeSettings& ghost_settings)
{
  ghost_sim_.enable(ghost_settings);
  ghost_info_.overlap_target=ghost_settings.exit_delay_when_overlap;
}

void Vehicle::disableGhostSim()
{
  ghost_sim_.disable();
}

btVector3 Vehicle::correctCOM(const btVector3& com,const VehicleMesh* mesh)
{
  btVector3 front_left_cs = mesh->getLocalWheelTransform(WheelPosition::FrontLeft).getOrigin();
  btVector3 front_right_cs = mesh->getLocalWheelTransform(WheelPosition::FrontRight).getOrigin();
  btVector3 rear_left_cs = mesh->getLocalWheelTransform(WheelPosition::RearLeft).getOrigin();
  btVector3 rear_right_cs = mesh->getLocalWheelTransform(WheelPosition::RearRight).getOrigin();
  btVector3 mid_front = front_left_cs + (front_right_cs - front_left_cs) / 2.f;
  btVector3 mid_rear = rear_left_cs + (rear_right_cs - rear_left_cs) / 2.f;

  btVector3 mid_point = mid_rear + (mid_front - mid_rear) / 2.f;
  mid_point.setY(0.f);

  btVector3 new_com=com;
  new_com+=mid_point;

  return new_com;
}

std::unique_ptr<btCompoundShape> Vehicle::createCollisonShape(const std::vector<btVector3>& vertices,
                                                              const btVector3& center_of_mass)
{
  std::unique_ptr<btCompoundShape> compound(new btCompoundShape());
  btConvexHullShape* hull = new btConvexHullShape();

  btScalar floor = -0.5;
  btScalar max_z=0;
  btScalar min_z=3.f;
  for ( btVector3 v : vertices)
  {
    if(v.z()>max_z)
      max_z=v.z();

    if(v.z()<min_z)
      min_z=v.z();

    if(v.y()<floor)
      v.setY(floor);

    hull->addPoint(v, false);
  }

  // 2. Inject 4 points at the front to FORCE it to be rectangular
  //btScalar fz = 2.6f; // Front-most Z
  btScalar hw = 0.9f; // Half-width
  btScalar hh = -0.3f; // Half-height

  hull->addPoint(btVector3( hw, hh,max_z)); // Top Right Front
  hull->addPoint(btVector3(-hw, hh,max_z)); // Top Left Front
  hull->addPoint(btVector3( hw, floor,min_z)); // Top Right Front
  hull->addPoint(btVector3(-hw, floor,min_z)); // Top Left Front

  hull->recalcLocalAabb();
  hull->optimizeConvexHull();
  hull->initializePolyhedralFeatures();

  btTransform localTransform;
  localTransform.setIdentity();
  localTransform.setOrigin(-center_of_mass);

  // The center of gravity of the compound shape is the origin.
  // When we add a rigidbody to the compound shape
  // it's center of gravity does not change.
  // This way we can add the chassis rigidbody one unit above our center of gravity
  // keeping it under our chassis, and not in the middle of it
  compound->addChildShape(localTransform, hull);

  return compound;
}

std::unique_ptr<btRigidBody> Vehicle::createRigidbody(btCompoundShape* col_shape,
      btMotionState* motion_state, btScalar mass)
{
  BOINK_ASSERT(mass != 0.f);

  btVector3 local_inertia(0, 0, 0);
  col_shape->calculateLocalInertia(mass, local_inertia);

  btRigidBody::btRigidBodyConstructionInfo rb_info(mass, motion_state, col_shape, local_inertia);

  std::unique_ptr<btRigidBody> body(new btRigidBody(rb_info));

  return body;
}

BoundingBox Vehicle::getBoundingDims(
    std::shared_ptr<btCollisionShape> col_shape)
{
  btVector3 aabb_min;
  btVector3 aabb_max;
  col_shape->getAabb(btTransform::getIdentity(),aabb_min,aabb_max);

  BoundingBox box;
  btScalar depth=aabb_max.z()-aabb_min.z();
  btScalar width=aabb_max.x()-aabb_min.x();

  box.top_left=aabb_max;
  box.top_left.setY(0);
  box.bottom_right=aabb_min;
  box.bottom_right.setY(0);
  
  box.bottom_left=box.bottom_right;
  box.bottom_left.setX(box.bottom_left.x()+width);

  box.top_right=box.bottom_right;
  box.top_right.setZ(box.bottom_right.z()+depth);

  return box;
}

void Vehicle::reset()
{
  rigidbody_->setLinearVelocity({0,0,0});
  rigidbody_->setAngularVelocity({0,0,0});

  rigidbody_->clearForces();

  // Prevent the 1-frame visual "swoosh" (Fixes interpolation artifacts)
  btTransform new_transform = rigidbody_->getWorldTransform();
  rigidbody_->setInterpolationWorldTransform(new_transform);
  rigidbody_->setInterpolationLinearVelocity(btVector3(0, 0, 0));
  rigidbody_->setInterpolationAngularVelocity(btVector3(0, 0, 0));

  vehicle_->reset();

  // After tp we cannot give vehicle better postion only worse
  btScalar new_coverage=
    track_->getRoad().
    getCoverage(this->getChassisWorldTransform().getOrigin());

  if(new_coverage-lap_info_.curr_lap_coverage>0.1)
    lap_info_.current_lap--;

  lap_info_.curr_lap_coverage=new_coverage;

  ghost_sim_.enterGhostModeForce();
}

int Vehicle::isVehicleOnTrack(bool max_lines) const
{
  btTransform trans=this->getChassisWorldTransform();
  btVector3 offset=center_of_mass_;

  return track_->getRoad().isObjectOnRoad(
      trans.getOrigin(),
      trans.getRotation(),
      offset,
      bounding_dimensions_,
      max_lines);
}

int Vehicle::isVehicleInPitstop(bool max_lines) const
{
  int wheel_sum=0;
  for(const auto& [zone_type,_]:track_->getPitstop().getZones())
    wheel_sum+=this->isVehicleInPitstop(zone_type,max_lines);

  BOINK_ASSERT(wheel_sum<=getNumWheels());

  return wheel_sum;
}

int Vehicle::isVehicleInPitstop(Pitstop::Zone zone, bool max_lines) const
{
  const Road& road=track_->getPitstop().getZone(zone);

  btTransform trans=this->getChassisWorldTransform();
  btVector3 offset=center_of_mass_;

  return road.isObjectOnRoad(
      trans.getOrigin(),
      trans.getRotation(),
      offset,
      bounding_dimensions_,
      max_lines);
}

bool Vehicle::isOverlapping() const
{
  const auto& overlap_vehicles = ghost_info_.overlap_vehicles;
  if (overlap_vehicles.empty()) 
    return false;

  return std::any_of(overlap_vehicles.begin(), overlap_vehicles.end(),
      [](const auto& pair)
      {
        return !pair.second.isRunning();
      });
}

bool Vehicle::isAnyOverlapTimerRunning() const
{
  const auto& overlap_vehicles = ghost_info_.overlap_vehicles;
  if (overlap_vehicles.empty()) 
    return false;

  return std::any_of(overlap_vehicles.begin(), overlap_vehicles.end(),
      [](const auto& pair)
      {
        return pair.second.isRunning();
      });
}

btScalar Vehicle::biggestLeftOverlapTime() const
{
  const auto& overlap_vehicles = ghost_info_.overlap_vehicles;
  if (overlap_vehicles.empty()) 
    return 0.f;

  auto it=std::min_element(overlap_vehicles.begin(),overlap_vehicles.end(),
      [](const auto& p0,const auto& p1)
      {
        return p0.second.getCurrent()<p1.second.getCurrent();
      });

  btScalar time_left=ghost_info_.overlap_target-it->second.getCurrent();
  return std::max(0.f,time_left);
}

bool Vehicle::hasStopped() const
{
  return this->getSpeed()<0.2f;
}

}  // namespace boink
