#pragma once

#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>

#include "boink/id_registry.h"
#include "boink/simulators/simulator.h"
#include "boink/gui/simulation_gui.h"
#include "boink/debugger/debugger.h"

#include <memory>
#include <piksel/gui_object.hh>

namespace boink
{
  class Simulation
  {
  public:
    Simulation(
        btScalar grafity_acceleration,
        Debugger* p_dbg=nullptr,
        std::shared_ptr<SimulationGui> gui=std::make_shared<SimulationGui>());
    Simulation(const Simulation&)=delete;
    Simulation(Simulation&&)=default;
    
    Simulation& operator=(const Simulation&)=delete;
    Simulation& operator=(Simulation&&)=delete;

    virtual ~Simulation() noexcept=default;

    Simulator::ID addSimulator(std::shared_ptr<Simulator> simulator);
    std::shared_ptr<const Simulator> getSimulator(Simulator::ID id) const;
    std::shared_ptr<Simulator> getSimulator(Simulator::ID id);
    bool removeSimulator(Simulator::ID id);
    size_t simulatorsSize() const {return simulators_.size();}

    btScalar getSimulationDuration() const { return simulation_duration_;}
    btScalar getGravitationalAcceleration() const;

    virtual int update(
        btScalar dt,
        int max_sub_steps=15,
        btScalar fixed_delta_time=1.f/120.f,
        btScalar max_delta_time=0.1);
    virtual void updateDebug();
  protected:
    std::shared_ptr<btDiscreteDynamicsWorld> getDynamicsWorld() 
    {
      return dynamics_world_;
    }
  private:
    void updateGui();
  protected:
    std::shared_ptr<SimulationGui> gui_;
    Debugger* p_dbg_=nullptr;
  private:
    std::unique_ptr<btDefaultCollisionConfiguration> collision_configuration_;
    std::unique_ptr<btCollisionDispatcher> dispatcher_;
    std::unique_ptr<btDbvtBroadphase> overlapping_pair_cache_;
    std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;

    std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world_;

    btScalar simulation_duration_=0;
    bool freeze_;
    IDRegistry<Simulator::ID,std::shared_ptr<Simulator>> simulators_;
  };
}
