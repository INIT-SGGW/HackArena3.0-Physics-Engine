// clang-format off
#pragma once

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btMotionState.h>

#include <memory>
#include <unordered_map>
#include <utility>

#include "boink/simulators/simulator.h"
#include "boink/simulators/track/track.h"
#include "boink/simulators/vehicle/physics/raycast_vehicle.h"
#include "boink/simulators/vehicle/physics/wheel_info.h"
#include "boink/simulators/vehicle/vehicle_mesh.h"
#include "boink/simulators/vehicle/wheel_position.h"
#include "boink/bullet_user_data.h"
#include "boink/simulators/vehicle/ghost_mode_settings.h"
#include "boink/simulators/vehicle/ghost_mode.h"
#include "boink/simulators/vehicle/lap_info.h"
#include "boink/bounding_box.h"

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
    
    enum class TurnDirection
    {
      Left,
      Right
    };

    struct GhostModeInfo
    {
      bool enabled=false;

      btScalar overlap_target=0.f;
      std::unordered_map<btCollisionObject*,Timer> overlap_vehicles;
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

    // Implementation must be done with care.
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
    void setVehicleToPitstop(Pitstop::Zone zone);

    const LapInfo& getLapInfo() const {return lap_info_;}

    btTransform getChassisWorldTransform() const;
    btScalar getChassisToGroundDist() const;

    const btTransform& getWheelWorldTransform(WheelPosition wheel_pos) const;
    btScalar getEngineRPM() const;
    int getCurrentGear() const;
    btScalar getWheelAngularSpeed(WheelPosition wheel_pos) const;
    const btTransform& getCenterOfMassTransform() const;

    UserData* getUserData() { return (UserData*)rigidbody_->getUserPointer();}

    btVector3 getVehicleDirection() const;

    btScalar getSpeed() const;
    btScalar getMass() const;
    int getNumWheels() const {return vehicle_->getNumWheels();}
    btVector3 getCenterOfMassCS() const;

    bool areAllWheelsOnGround() const;

    btScalar getTyreHealth(WheelPosition pos) const;
    WheelInfo::TyreType getTyreType(WheelPosition pos) const;
    btScalar getTyreTempCelsius(WheelPosition pos) const;
    btScalar getTyreSlipLen(WheelPosition pos) const;

    void setTuning(const RaycastVehicle::VehicleTuning& tuning);
    const RaycastVehicle::VehicleTuning& getTuning() const;

    std::pair<btScalar,TurnDirection> getSteering(
        WheelPosition pos) const;

    // Value from [0,1]
    void setSteering(btScalar value, TurnDirection dir);
    void setEngineForce(btScalar force);
    void setBrake(btScalar brake);
    void setBrakeBias(btScalar bias);
    void setDiffSetting(btScalar diffsetting);
    void setTyreType(WheelInfo::TyreType tyre_type);

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
    BoundingBox getBoundingDims() const { return bounding_dimensions_;}
    
    /**
     * @brief Resets all speeds, forces, interpolation of a vehicle.
     */
    void reset();

    int isVehicleOnTrack(bool max_lines=false) const;
    int isVehicleInPitstop(bool max_lines=false) const;
    int isVehicleInPitstop(Pitstop::Zone zone, bool max_lines=false) const;

    bool isOverlapping() const;
    bool isAnyOverlapTimerRunning() const;
    btScalar biggestLeftOverlapTime() const;

    bool hasStopped() const;
  private:
    void updateLapInfo(btScalar dt);
    void updatePitstop(btScalar dt);
  private:
    static btVector3 correctCOM(const btVector3& COM, const VehicleMesh* mesh);
    static std::unique_ptr<btCompoundShape> createCollisonShape(
        const std::vector<btVector3>& vertices,
        const btVector3& center_of_mass);
    static std::unique_ptr<btRigidBody> createRigidbody(
        btCompoundShape* col_shape,
        btMotionState* motion_state,
        btScalar mass);

    static BoundingBox getBoundingDims(std::shared_ptr<btCollisionShape> col_shape);
  private:
    static constexpr btScalar kMaxFixZoneSpeed=15.f;
    static constexpr btScalar kMaxFixZonePenaltySpeed=5.f;
    static constexpr btScalar kBrakingDuration=5.f;
    static constexpr btScalar kPitstopBrakingForce=50000.f;
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

    /// <summary>
    /// In radians.
    /// </summary>
    btScalar max_steer_angle_;
    RaycastVehicle::VehicleTuning tuning_;
    
    LapInfo lap_info_;

    GhostModeInfo ghost_info_;
    GhostMode ghost_sim_;

    BoundingBox bounding_dimensions_;
    Timer pitstop_timer_;

    UserData user_data_;
    std::shared_ptr<VehicleGui> gui_;
};
}  // namespace boink
