#include "boink/simulation/simulation.h"

#include <piksel/gui_object.hh>

namespace boink
{
  Simulation::Simulation(
      btScalar gravity_acceleration,
      Debugger* p_dbg,
      std::shared_ptr<SimulationGui> gui)
    :gui_(gui),
    p_dbg_(p_dbg),
    collision_configuration_(new btDefaultCollisionConfiguration()),
    dispatcher_(new btCollisionDispatcher(collision_configuration_.get())),
    overlapping_pair_cache_(new btDbvtBroadphase()),
    solver_(new btSequentialImpulseConstraintSolver()),
    dynamics_world_(new btDiscreteDynamicsWorld(
          dispatcher_.get(),overlapping_pair_cache_.get(),
          solver_.get(),collision_configuration_.get()))
  {
    dynamics_world_->setGravity(btVector3(0, -gravity_acceleration, 0));

    if(p_dbg_)
    {
      freeze_=true;
      dynamics_world_->setDebugDrawer(p_dbg_->getRendererPtr());
      p_dbg_->addGui(gui_);
    }
    else
      freeze_=false;

    gui_->duration=&simulation_duration_;
    gui_->freeze=&freeze_;
  } 

  Simulator::ID Simulation::addSimulator(std::shared_ptr<Simulator> simulator)
  {
    auto id=simulators_.add(simulator);
    gui_->addSimulatorGui(id,simulator->getGui());

    return id;
  }

  std::shared_ptr<const Simulator> Simulation::getSimulator(Simulator::ID id) const
  {
    return simulators_.at(id);
  }

  std::shared_ptr<Simulator> Simulation::getSimulator(Simulator::ID id) 
  {
    return simulators_.at(id);
  }

  bool Simulation::removeSimulator(Simulator::ID id)
  {
    gui_->removeSimulatorGui(id);
    return simulators_.remove(id);
  }

  btScalar Simulation::getGravitationalAcceleration() const
  {
    return dynamics_world_->getGravity().y();
  }

  int Simulation::update(
      btScalar dt,
      int max_sub_steps,
      btScalar fixed_delta_time,
      btScalar max_delta_time)
  {
    if(freeze_)
      return 0;

    // With large delta time simulation behaves oddly.
    dt=btMin(dt,max_delta_time);

    int steps=dynamics_world_->stepSimulation(dt,max_sub_steps,fixed_delta_time);
    btScalar simulation_step=steps*fixed_delta_time;
    simulation_duration_+=simulation_step;

    for(auto& id:simulators_)
    {
      auto& simulator=simulators_.at(id);
      simulator->update(simulation_step);
    }

    return steps;
  }

  void Simulation::updateDebug()
  {
    if(!p_dbg_)
      return;

    dynamics_world_->debugDrawWorld();

    for(auto& id:simulators_)
    {
      auto& simulator=simulators_.at(id);
      simulator->updateRender(p_dbg_->getRendererPtr());
    }

    this->updateGui();
  }

  void Simulation::updateGui()
  {
    if(!gui_)
      return;

    gui_->gravity_acc=this->getGravitationalAcceleration();
    gui_->num_simulators=this->simulatorsSize();
  }
}
