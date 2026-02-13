#include "boink/simulation/simulation.h"

namespace boink
{
  Simulation::Simulation(btScalar gravity_acceleration)
    :collision_configuration_(new btDefaultCollisionConfiguration()),
    dispatcher_(new btCollisionDispatcher(collision_configuration_.get())),
    overlapping_pair_cache_(new btDbvtBroadphase()),
    solver_(new btSequentialImpulseConstraintSolver()),
    dynamics_world_(new btDiscreteDynamicsWorld(
          dispatcher_.get(),overlapping_pair_cache_.get(),
          solver_.get(),collision_configuration_.get()))
  {
    dynamics_world_->setGravity(btVector3(0, -gravity_acceleration, 0));
  } 

  btScalar Simulation::getGravitationalAcceleration() const
  {
    return dynamics_world_->getGravity().y();
  }

  void Simulation::registerDebugger(Debugger* p_dbg)
  {
    p_dbg_=p_dbg;
    dynamics_world_->setDebugDrawer(p_dbg_->getRendererPtr());
  }

  void Simulation::update(
      btScalar dt,
      int max_sub_steps,
      btScalar fixed_delta_time,
      btScalar max_delta_time)
  {
    // With large delta time simulation behaves oddly.
    dt=btMin(dt,max_delta_time);

    int steps=dynamics_world_->stepSimulation(dt,max_sub_steps,fixed_delta_time);
    btScalar simulation_step=steps*fixed_delta_time;
    simulation_duration_+=simulation_step;

    dynamics_world_->debugDrawWorld();

    for(auto& id:simulators_)
    {
      auto& simulator=simulators_.at(id);
      simulator->update(simulation_step);
      if(p_dbg_)
        simulator->updateDebug(p_dbg_);
    }
  }
}
