#include "boink/world.h"

namespace boink
{
  World::World(const CarModel& car_model)
    :car_manager(car_model)
  {}

  void World::setDebuger(DebugRender* dbg)
  {
    simulation.registerDebugDrawer(dbg);
  }

  void World::start(double dt)
  {
    car_manager.SetupSystems(dt);
    time_passed_+=dt;
  }
  void World::update(double dt)
  {
    car_manager.UpdateSystems(dt);
    simulation.step(dt);
    time_passed_+=dt;
  }

  void World::addGround(const btVector3& dims, const btVector3& pos)
  {
    simulation.addGround(dims,pos);
  }
  void World::addSphere(btScalar radius, const btVector3& pos)
  {
    simulation.addSphere(radius,pos);
  }
}
