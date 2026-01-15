#pragma once

#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "LinearMath/btIDebugDraw.h"
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>
#include <BulletDynamics/Vehicle/btVehicleRaycaster.h>

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
    typedef unsigned int ObjectID;
  public:
    Simulation(std::string_view track_filepath);
    Simulation(const Simulation&)=delete;
    Simulation(Simulation&&)=default;
    
    Simulation& operator=(const Simulation&)=delete;
    Simulation& operator=(Simulation&&)=default;

    ~Simulation() noexcept=default;

    ObjectID addCar(const Vehicle::CreationInfo& info);
    void removeCar(ObjectID id);
    Vehicle& getCar(ObjectID id);
    size_t getCarNumber() const { return vehicles_.size();}

    Track& getTrack();

    void addSphere(btScalar radius, const btVector3& pos);

    void step(double dt);
    void registerDebugDrawer(btIDebugDraw* dbg);
  public:
    static constexpr double GRAVITATIONAL_ACCELERATION=10.;
  private:
    std::unique_ptr<btDefaultCollisionConfiguration> collision_configuration_;
    std::unique_ptr<btCollisionDispatcher> dispatcher_;
    std::unique_ptr<btDbvtBroadphase> overlapping_pair_cache_;
    std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;

    std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world_;

    Track track_;
    std::vector<Vehicle> vehicles_;
  };
}
