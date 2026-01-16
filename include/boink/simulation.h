#pragma once

#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "LinearMath/btIDebugDraw.h"

#include "boink/simulation/track.h"
#include "boink/simulation/vehicle.h"

#include <memory>
#include <string_view>
#include <vector>

namespace boink
{
  class Simulation
  {
  public:
    typedef uint64_t ObjectID;
  public:
    Simulation(std::string_view track_filepath);
    Simulation(const Simulation&)=delete;
    Simulation(Simulation&&)=default;
    
    Simulation& operator=(const Simulation&)=delete;
    Simulation& operator=(Simulation&&)=default;

    ~Simulation() noexcept=default;

    ObjectID addVehicle(const Vehicle::CreationInfo& info);
    void removeVehicle(ObjectID id);
    Vehicle& getVehicle(ObjectID id);
    size_t getVehicleNumber() const { return vehicles_.size();}

    Track& getTrack();

    void step(btScalar dt) noexcept;
    btScalar getSimulationDuration() const { return simulation_duration_;}
    void registerDebugDrawer(btIDebugDraw* dbg);
  public:
    static constexpr btScalar kGravitationalAcceleration=10.;
    static constexpr btScalar kMaxDeltaTime=0.1;
    static constexpr btScalar kFixedDeltaTime=1.f/60.f;
    static constexpr int kMaxSubSteps=10;
  private:
    std::unique_ptr<btDefaultCollisionConfiguration> collision_configuration_;
    std::unique_ptr<btCollisionDispatcher> dispatcher_;
    std::unique_ptr<btDbvtBroadphase> overlapping_pair_cache_;
    std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;

    std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world_;

    Track track_;
    std::vector<Vehicle> vehicles_;
    btScalar simulation_duration_=0;
  };
}
