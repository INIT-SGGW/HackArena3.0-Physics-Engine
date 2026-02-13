#pragma once

#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>

#include "boink/id_registry.h"
#include "boink/simulators/simulator.h"
#include "boink/debugger/debugger.h"

#include <memory>

namespace boink
{
  class Simulation
  {
  public:
    Simulation(btScalar grafity_acceleration=9.81f);
    Simulation(const Simulation&)=delete;
    Simulation(Simulation&&)=default;
    
    Simulation& operator=(const Simulation&)=delete;
    Simulation& operator=(Simulation&&)=delete;

    virtual ~Simulation() noexcept=default;

    const IDRegistry<Simulator::ID,std::shared_ptr<Simulator>>&
      getSimulators() const {return simulators_;}
    IDRegistry<Simulator::ID,std::shared_ptr<Simulator>>&
      getSimulators() {return simulators_;}

    btScalar getSimulationDuration() const { return simulation_duration_;}
    btScalar getGravitationalAcceleration() const;
    void registerDebugger(Debugger* p_dbg);

    virtual void update(
        btScalar dt,
        int max_sub_steps=10,
        btScalar fixed_delta_time=1.f/120.f,
        btScalar max_delta_time=0.1);
  protected:
    std::shared_ptr<btDiscreteDynamicsWorld> getDynamicsWorld() 
    {
      return dynamics_world_;
    }
  private:
    std::unique_ptr<btDefaultCollisionConfiguration> collision_configuration_;
    std::unique_ptr<btCollisionDispatcher> dispatcher_;
    std::unique_ptr<btDbvtBroadphase> overlapping_pair_cache_;
    std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;

    std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world_;

    btScalar simulation_duration_=0;
    IDRegistry<Simulator::ID,std::shared_ptr<Simulator>> simulators_;

    Debugger* p_dbg_;
  };
}
