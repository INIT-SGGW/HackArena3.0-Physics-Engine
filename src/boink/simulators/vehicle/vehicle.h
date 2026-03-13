// clang-format off
#pragma once

#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btMotionState.h>

#include <memory>

#include "boink/simulators/simulator.h"
#include "boink/simulators/track/track.h"
#include "boink/simulators/vehicle/physics/raycast_vehicle.h"
#include "boink/simulators/vehicle/physics/wheel_info.h"
#include "boink/simulators/vehicle/vehicle_mesh.h"
#include "boink/simulators/vehicle/wheel_position.h"
#include "boink/bullet_user_data.h"
#include "boink/simulators/vehicle/ghost_mode_settings.h"
#include "boink/simulators/vehicle/ghost_mode.h"

namespace boink
{
class VehicleGui;
class Vehicle : public Simulator
{
  friend class VehicleGui;
  public:
    struct CreationInfo
    {
      std::shared_ptr<const VehicleMesh> mesh;
      btScalar mass;
      btScalar wheel_radius;
      btScalar suspension_rest_length;
      btScalar max_steer_angle;
      btVector3 center_of_mass;
      RaycastVehicle::VehicleTuning tuning;
      WheelInfo::TyreType tyre_type;
    };

    struct Dimensions
    {
      btScalar width;
      btScalar height;
      btScalar depth;
    };
    
    enum class TurnDirection
    {
      Left,
      Right
    };

    struct GhostModeInfo
    {
      bool enabled=false;
    };
    struct UserData : public BulletUserData
    {
      UserData(GhostModeInfo* ghost_info)
        :BulletUserData(Type::Vehicle),
        ghost_info(ghost_info)
      {}

      GhostModeInfo* ghost_info;
    };
  public:
    Vehicle(
        const CreationInfo& create_info,
        std::shared_ptr<const Track> track,
        std::shared_ptr<btDynamicsWorld> world);

    Vehicle(const Vehicle&)=delete;
    Vehicle& operator=(const Vehicle&)=delete;

    // if custom need remember
    // to set for rigdbody user ptr new ptr to new user data
    Vehicle(Vehicle&&)noexcept=delete;
    Vehicle& operator=(Vehicle&&)noexcept=delete;

    ~Vehicle() noexcept;

    void update(btScalar dt) override;
    void updateRender(Renderer* renderer) override;
    std::shared_ptr<piksel::GuiObject> getGui() override;

    void setChassisWorldTransform(const btTransform& transform);

    int getLapsCompleted() const {return laps_completed_;}
    btScalar getCurrentLapDistanceCovered() const {return curr_lap_dist_point_;}

    btTransform getChassisWorldTransform() const;
    btScalar getChassisToGroundDist() const;

    const btTransform& getWheelWorldTransform(WheelPosition wheel_pos) const;
    btScalar getEngineRPM() const;
    int getCurrentGear() const;
    btScalar getWheelAngularSpeed(WheelPosition wheel_pos) const;
    const btTransform& getCenterOfMassTransform() const;

    btScalar getSpeed() const;
    btScalar getMass() const;
    btVector3 getCenterOfMassCS() const;

    btScalar getTyreHealth(WheelPosition pos) const;
    WheelInfo::TyreType getTyreType(WheelPosition pos) const;
    btScalar getTyreTempCelsius(WheelPosition pos) const;

    void setTuning(const RaycastVehicle::VehicleTuning& tuning);
    const RaycastVehicle::VehicleTuning& getTuning() const;

    // Value from [0,1]
    void setSteering(btScalar value, TurnDirection dir);
    void setEngineForce(btScalar force);
    void setBrake(btScalar brake);

    /// <summary>
    /// Sets gear up.
    /// </summary>
    /// <returns>Whether gear was really upped.</returns>
    bool setGearUp();

    /// <summary>
    /// Sets gear down.
    /// </summary>
    /// <returns>Whether gear was really downed.</returns>
    bool setGearDown();

    void enableGhostSim(const GhostModeSettings& ghost_setttings);
    void disableGhostSim();
    bool isInGhostMode() const {return ghost_info_.enabled;}

    const GhostMode& getGhostMode() const {return ghost_sim_;}
    Dimensions getBoundingDims() const { return bounding_dimensions_;}
  private:
    static btVector3 correctCOM(const btVector3& COM, const VehicleMesh* mesh);
    static std::unique_ptr<btCompoundShape> createCollisonShape(
        const std::vector<btVector3>& vertices,
        const btVector3& center_of_mass);
    static std::unique_ptr<btRigidBody> createRigidbody(
        btCompoundShape* col_shape,
        btMotionState* motion_state,
        btScalar mass);

    static Dimensions getBoundingDims(std::shared_ptr<btCollisionShape> col_shape);
  private:
    std::shared_ptr<const VehicleMesh> mesh_;
    std::shared_ptr<btDynamicsWorld> world_;

    btVector3 center_of_mass_;

    std::shared_ptr<btCompoundShape> collision_shape_;
    std::unique_ptr<btMotionState> motion_state_;
    std::unique_ptr<btRigidBody> rigidbody_;
    std::unique_ptr<VehicleRaycaster> raycaster_;
    std::unique_ptr<RaycastVehicle> vehicle_;

    std::shared_ptr<const Track> track_;

    btScalar max_steer_angle_;
    RaycastVehicle::VehicleTuning tuning_;
    
    int laps_completed_;
    btScalar curr_lap_dist_point_;

    GhostModeInfo ghost_info_;
    GhostMode ghost_sim_;

    Dimensions bounding_dimensions_;

    UserData user_data_;
    std::shared_ptr<VehicleGui> gui_;
};
}  // namespace boink
